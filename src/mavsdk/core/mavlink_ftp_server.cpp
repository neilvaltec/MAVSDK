#include "mavlink_ftp_server.h"
#include "server_component_impl.h"
#include <fstream>

#if defined(WINDOWS)
#include "tronkko_dirent.h"
#include "stackoverflow_unistd.h"
#else
#include <dirent.h>
#include <unistd.h>
#endif
#include <fcntl.h>
#include <sys/stat.h>

#include "unused.h"
#include "crc32.h"
#include "fs.h"
#include <algorithm>
#include <future>

namespace mavsdk {

MavlinkFtpServer::MavlinkFtpServer(ServerComponentImpl& server_component_impl) :
    _server_component_impl(server_component_impl)
{
    if (const char* env_p = std::getenv("MAVSDK_FTP_DEBUGGING")) {
        if (std::string(env_p) == "1") {
            LogDebug() << "Ftp debugging is on.";
            _debugging = true;
        }
    }

    _server_component_impl.register_mavlink_message_handler(
        MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
        [this](const mavlink_message_t& message) { process_mavlink_ftp_message(message); },
        this);
}

void MavlinkFtpServer::process_mavlink_ftp_message(const mavlink_message_t& msg)
{
    bool stream_send = false;
    mavlink_file_transfer_protocol_t ftp_req;
    mavlink_msg_file_transfer_protocol_decode(&msg, &ftp_req);

    if (_debugging) {
        LogDebug() << "Processing FTP message to target compid: "
                   << std::to_string(ftp_req.target_component) << ", our compid: "
                   << std::to_string(_server_component_impl.get_own_component_id());
    }

    if (ftp_req.target_system != 0 &&
        ftp_req.target_system != _server_component_impl.get_own_system_id()) {
        LogWarn() << "wrong sysid!";
        return;
    }

    if (ftp_req.target_component != 0 &&
        ftp_req.target_component != _server_component_impl.get_own_component_id()) {
        LogWarn() << "wrong compid!";
        return;
    }

    PayloadHeader* payload = reinterpret_cast<PayloadHeader*>(&ftp_req.payload[0]);

    ServerResult error_code = ServerResult::SUCCESS;

    // Basic sanity check: validate length before use.
    if (payload->size > max_data_length) {
        error_code = ServerResult::ERR_INVALID_DATA_SIZE;
    } else {
        if (_debugging) {
            LogDebug() << "FTP opcode: " << (int)payload->opcode << ", size: " << (int)payload->size
                       << ", offset: " << (int)payload->offset << ", seq: " << payload->seq_number;
        }

        if (_curr_op != CMD_NONE && _curr_op != payload->req_opcode) {
            LogWarn() << "Received ACK not matching our current operation, resetting";
            _reset();
        }

        _target_system_id = msg.sysid;
        _target_component_id = msg.compid;

        switch (payload->opcode) {
            case CMD_NONE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_NONE";
                }
                break;

            case CMD_TERMINATE_SESSION:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_TERMINATE_SESSION";
                }
                error_code = _work_terminate(payload);
                break;

            case CMD_RESET_SESSIONS:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_RESET_SESSIONS";
                }
                error_code = _work_reset(payload);
                break;

            case CMD_LIST_DIRECTORY:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_LIST_DIRECTORY";
                }
                error_code = _work_list(payload);
                break;

            case CMD_OPEN_FILE_RO:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_OPEN_FILE_RO";
                }
                error_code = _work_open_file_readonly(payload);
                break;

            case CMD_CREATE_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_CREATE_FILE";
                }
                error_code = _work_create_file(payload);
                break;

            case CMD_OPEN_FILE_WO:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_OPEN_FILE_WO";
                }
                error_code = _work_open_file_writeonly(payload);
                break;

            case CMD_READ_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_READ_FILE";
                }
                error_code = _work_read(payload);
                break;

            case CMD_BURST_READ_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_BURST_READ_FILE";
                }
                error_code = _work_burst(payload);
                stream_send = true;
                break;

            case CMD_WRITE_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_WRITE_FILE";
                }
                error_code = _work_write(payload);
                break;

            case CMD_REMOVE_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_REMOVE_FILE";
                }
                error_code = _work_remove_file(payload);
                break;

            case CMD_RENAME:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_RENAME";
                }
                error_code = _work_rename(payload);
                break;

            case CMD_CREATE_DIRECTORY:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_CREATE_DIRECTORY";
                }
                error_code = _work_create_directory(payload);
                break;

            case CMD_REMOVE_DIRECTORY:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_REMOVE_DIRECTORY";
                }
                error_code = _work_remove_directory(payload);
                break;

            case CMD_CALC_FILE_CRC32:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_CALC_FILE_CRC32";
                }
                error_code = _work_calc_file_CRC32(payload);
                break;

            default:
                // Not for us, ignore it.
                return;
        }
    }

    if (_debugging) {
        LogDebug() << "Error code: " << std::to_string(error_code);
    }

    payload->seq_number++;

    // handle success vs. error
    if (error_code == ServerResult::SUCCESS) {
        payload->req_opcode = payload->opcode;
        payload->opcode = RSP_ACK;

    } else {
        uint8_t r_errno = errno;
        payload->req_opcode = payload->opcode;
        payload->opcode = RSP_NAK;
        payload->size = 1;

        if (r_errno == EEXIST) {
            error_code = ServerResult::ERR_FAIL_FILE_EXISTS;
        } else if (r_errno == ENOENT && error_code == ServerResult::ERR_FAIL_ERRNO) {
            error_code = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        }

        *reinterpret_cast<ServerResult*>(payload->data) = error_code;

        if (error_code == ServerResult::ERR_FAIL_ERRNO) {
            payload->size = 2;
            *reinterpret_cast<uint8_t*>(payload->data + 1) = r_errno;
        }
    }

    _last_reply_valid = false;

    // Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
    if (!stream_send || error_code != ServerResult::SUCCESS) {
        // keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS
        // resends the request, we can simply resend the response.
        _last_reply_valid = true;
        _last_reply_seq = payload->seq_number;

        mavlink_msg_file_transfer_protocol_pack(
            _server_component_impl.get_own_system_id(),
            _server_component_impl.get_own_component_id(),
            &_last_reply,
            _network_id,
            _target_system_id,
            _target_component_id,
            reinterpret_cast<const uint8_t*>(payload));
        _server_component_impl.send_message(_last_reply);
    }
}

MavlinkFtpServer::~MavlinkFtpServer()
{
    _reset();
}

void MavlinkFtpServer::_end_read_session(bool delete_file)
{
    _curr_op = CMD_NONE;
    if (_ofstream.stream.is_open()) {
        _ofstream.stream.close();

        if (delete_file) {
            fs_remove(_ofstream.path);
        }
    }
    _terminate_session();
}

void MavlinkFtpServer::_read()
{
    if (_bytes_transferred >= _file_size) {
        _session_result = ServerResult::SUCCESS;
        _end_read_session();
        return;
    }

    auto payload = PayloadHeader{};
    payload.seq_number = _seq_number++;
    payload.session = _session;
    payload.opcode = _curr_op = CMD_READ_FILE;
    payload.offset = _bytes_transferred;
    payload.size =
        std::min(static_cast<uint32_t>(max_data_length), _file_size - _bytes_transferred);
    _send_mavlink_ftp_message(payload);
}

void MavlinkFtpServer::_end_write_session()
{
    _curr_op = CMD_NONE;
    if (_ifstream) {
        _ifstream.close();
    }
    _terminate_session();
}

void MavlinkFtpServer::_terminate_session()
{
    if (!_session_valid) {
        return;
    }
    auto payload = PayloadHeader{};
    payload.seq_number = _seq_number++;
    payload.session = _session;
    payload.opcode = _curr_op = CMD_TERMINATE_SESSION;
    payload.offset = 0;
    payload.size = 0;
    _send_mavlink_ftp_message(payload);
}

void MavlinkFtpServer::_list_directory(uint32_t offset)
{
    auto payload = PayloadHeader{};
    payload.seq_number = _seq_number++;
    payload.session = 0;
    payload.opcode = _curr_op = CMD_LIST_DIRECTORY;
    payload.offset = offset;
    strncpy(reinterpret_cast<char*>(payload.data), _last_path.c_str(), max_data_length - 1);
    payload.size = _last_path.length() + 1;

    if (offset == 0) {
        _curr_directory_list.clear();
    }
    _send_mavlink_ftp_message(payload);
}

void MavlinkFtpServer::_send_mavlink_ftp_message(const PayloadHeader& payload)
{
    mavlink_msg_file_transfer_protocol_pack(
        _server_component_impl.get_own_system_id(),
        _server_component_impl.get_own_component_id(),
        &_last_command,
        _network_id,
        _target_system_id,
        _target_component_id,
        reinterpret_cast<const uint8_t*>(&payload));
    _server_component_impl.send_message(_last_command);
}

/// @brief Guarantees that the payload data is null terminated.
/// @return Returns payload data as a std string
std::string MavlinkFtpServer::_data_as_string(PayloadHeader* payload)
{
    // guarantee null termination
    if (payload->size < max_data_length) {
        payload->data[payload->size] = '\0';

    } else {
        payload->data[max_data_length - 1] = '\0';
    }

    // and return data
    return std::string(reinterpret_cast<char*>(&(payload->data[0])));
}

std::string MavlinkFtpServer::_get_path(PayloadHeader* payload)
{
    return _get_path(_data_as_string(payload));
}

void MavlinkFtpServer::set_root_directory(const std::string& root_dir)
{
    _root_dir = fs_canonical(root_dir);
}

std::string MavlinkFtpServer::_get_path(const std::string& payload_path)
{
    return fs_canonical(_root_dir + path_separator + payload_path);
}

std::string MavlinkFtpServer::_get_rel_path(const std::string& path)
{
    return path.substr(_root_dir.length());
}

MavlinkFtpServer::ServerResult
MavlinkFtpServer::_work_list(PayloadHeader* payload, bool list_hidden)
{
    if (_root_dir.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    ServerResult error_code = ServerResult::SUCCESS;

    uint8_t offset = 0;
    // move to the requested offset
    uint32_t requested_offset = payload->offset;

    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL_FILE_PROTECTED;
    }
    if (!fs_exists(path)) {
        LogWarn() << "FTP: can't open path " << path;
        // this is not an FTP error, abort directory by simulating eof
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    struct dirent* dp;
    DIR* dfd = opendir(path.c_str());
    if (dfd != nullptr) {
        while ((dp = readdir(dfd)) != nullptr) {
            if (requested_offset > 0) {
                requested_offset--;
                continue;
            }

            std::string filename = dp->d_name;
            std::string full_path = path + path_separator + filename;

            std::string entry_s = DIRENT_SKIP;
            if (list_hidden || filename.rfind(".", 0) != 0) {
#ifdef _DIRENT_HAVE_D_TYPE
                bool type_reg = (dp->d_type == DT_REG);
                bool type_dir = (dp->d_type == DT_DIR);
#else
                struct stat statbuf;
                stat(full_path.c_str(), &statbuf);
                bool type_reg = S_ISREG(statbuf.st_mode);
                bool type_dir = S_ISDIR(statbuf.st_mode);
#endif
                if (type_reg) {
                    entry_s = DIRENT_FILE + _get_rel_path(full_path) + "\t" +
                              std::to_string(fs_file_size(full_path));
                } else if (type_dir) {
                    entry_s = DIRENT_DIR + _get_rel_path(full_path);
                }
            }

            // Do we have room for the dir entry and the null terminator?
            if (offset + entry_s.length() + 1 > max_data_length) {
                break;
            }
            uint8_t len = static_cast<uint8_t>(entry_s.length() + 1);
            memcpy(&payload->data[offset], entry_s.c_str(), len);

            offset += len;
        }
        closedir(dfd);
    }

    LogWarn() << "Setting offset: " << (int)offset;

    if (offset == 0) {
        // We are done and need to respond with EOF.
        error_code = ServerResult::ERR_EOF;
        payload->size = 1;
        // FIXME: don't read wrong errno downstream.
        errno = 0;

    } else {
        payload->size = offset;
    }

    return error_code;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_open_file_readonly(PayloadHeader* payload)
{
    if (_session_info.fd >= 0) {
        _reset();
    }

    std::string path = [payload, this]() {
        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        const auto it = _tmp_files.find(_data_as_string(payload));
        if (it != _tmp_files.end()) {
            return it->second;
        } else {
            return _root_dir.empty() ? "" : _get_path(payload);
        }
    }();

    if (path.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    if (_debugging) {
        LogInfo() << "Finding " << path << " in " << _root_dir;
    }
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL;
    }

    if (_debugging) {
        LogDebug() << "Going to open readonly: " << path;
    }

    if (!fs_exists(path)) {
        LogWarn() << "FTP: Open failed - file not found";
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    uint32_t file_size = fs_file_size(path);

    int fd = ::open(path.c_str(), O_RDONLY);

    if (fd < 0) {
        LogWarn() << "FTP: Open failed";
        return ServerResult::ERR_FAIL;
    }

    _session_info.fd = fd;
    _session_info.file_size = file_size;
    _session_info.stream_download = false;

    payload->session = 0;
    payload->size = sizeof(uint32_t);
    memcpy(payload->data, &file_size, payload->size);

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_open_file_writeonly(PayloadHeader* payload)
{
    if (_session_info.fd >= 0) {
        _reset();
    }

    std::string path = [payload, this]() {
        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        const auto it = _tmp_files.find(_data_as_string(payload));
        if (it != _tmp_files.end()) {
            return it->second;
        } else {
            return _root_dir.empty() ? "" : _get_path(payload);
        }
    }();

    if (path.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    if (_debugging) {
        LogInfo() << "Finding " << path << " in " << _root_dir;
    }
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL;
    }

    if (_debugging) {
        LogDebug() << "Going to open writeonly: " << path;
    }

    // fail only if requested open for read
    if (!fs_exists(path)) {
        LogWarn() << "FTP: Open failed - file not found";
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    uint32_t file_size = fs_file_size(path);

    int fd = ::open(path.c_str(), O_WRONLY, 0666);

    if (fd < 0) {
        LogWarn() << "FTP: Open failed";
        return (errno == ENOENT) ? ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST :
                                   ServerResult::ERR_FAIL;
    }

    _session_info.fd = fd;
    _session_info.file_size = file_size;
    _session_info.stream_download = false;

    payload->session = 0;
    payload->size = sizeof(uint32_t);
    memcpy(payload->data, &file_size, payload->size);

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_create_file(PayloadHeader* payload)
{
    if (_session_info.fd >= 0) {
        _reset();
    }

    std::string path = [payload, this]() {
        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        const auto it = _tmp_files.find(_data_as_string(payload));
        if (it != _tmp_files.end()) {
            return it->second;
        } else {
            return _root_dir.empty() ? "" : _get_path(payload);
        }
    }();

    if (path.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    if (_debugging) {
        LogInfo() << "Finding " << path << " in " << _root_dir;
    }
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL;
    }

    if (_debugging) {
        LogDebug() << "Creating file: " << path;
    }

    int fd = ::open(path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0666);

    if (fd < 0) {
        LogWarn() << "FTP: Open failed";
        return ServerResult::ERR_FAIL;
    }

    _session_info.fd = fd;
    _session_info.file_size = 0;

    payload->session = 0;
    payload->size = 0;

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_read(PayloadHeader* payload)
{
    if (payload->session != 0 || _session_info.fd < 0) {
        _reset();
        // return ServerResult::ERR_INVALID_SESSION;
    }

    // We have to test seek past EOF ourselves, lseek will allow seek past EOF
    LogWarn() << "Offset!" << payload->offset << ", size: " << _session_info.file_size;
    if (payload->offset >= _session_info.file_size) {
        return ServerResult::ERR_EOF;
    }

    if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
        LogWarn() << "fail!";
        return ServerResult::ERR_FAIL;
    }

    auto bytes_read = ::read(_session_info.fd, &payload->data[0], max_data_length);
    LogErr() << "Read " << bytes_read << " of limit " << std::to_string(max_data_length);

    if (bytes_read < 0) {
        // Negative return indicates error other than eof
        return ServerResult::ERR_FAIL;
    }

    payload->size = bytes_read;

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_burst(PayloadHeader* payload)
{
    if (payload->session != 0 && _session_info.fd < 0) {
        _reset();
        // return ServerResult::ERR_INVALID_SESSION;
    }

    // Setup for streaming sends
    _session_info.stream_download = true;
    _session_info.stream_offset = payload->offset;
    _session_info.stream_chunk_transmitted = 0;
    _session_info.stream_seq_number = payload->seq_number + 1;
    _session_info.stream_target_system_id = 0; // FIXME: _server_component_impl.get_system_id();

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_write(PayloadHeader* payload)
{
    if (payload->session != 0 && _session_info.fd < 0) {
        _reset();
        // return ServerResult::ERR_INVALID_SESSION;
    }

    if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
        // Unable to see to the specified location
        return ServerResult::ERR_FAIL;
    }

    static unsigned all_written = 0;
    int bytes_written = ::write(_session_info.fd, &payload->data[0], payload->size);
    all_written += bytes_written;

    LogWarn() << "Written: " << bytes_written << " of " << all_written;

    if (bytes_written < 0) {
        // Negative return indicates error other than eof
        return ServerResult::ERR_FAIL;
    }

    payload->size = sizeof(uint32_t);
    memcpy(payload->data, &bytes_written, payload->size);

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_terminate(PayloadHeader* payload)
{
    UNUSED(payload);
    _reset();

    return ServerResult::SUCCESS;
}

void MavlinkFtpServer::_reset()
{
    if (_session_info.fd != -1) {
        close(_session_info.fd);
        _session_info.fd = -1;
        _session_info.stream_download = false;
    }
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_reset(PayloadHeader* payload)
{
    UNUSED(payload);
    _reset();

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_remove_directory(PayloadHeader* payload)
{
    if (_root_dir.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL;
    }

    if (!fs_exists(path)) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }
    if (fs_remove(path)) {
        return ServerResult::SUCCESS;
    } else {
        return ServerResult::ERR_FAIL;
    }
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_create_directory(PayloadHeader* payload)
{
    if (_root_dir.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL;
    }

    if (fs_exists(path)) {
        return ServerResult::ERR_FAIL_FILE_EXISTS;
    }
    if (fs_create_directory(path)) {
        return ServerResult::SUCCESS;
    } else {
        return ServerResult::ERR_FAIL_ERRNO;
    }
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_remove_file(PayloadHeader* payload)
{
    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        return ServerResult::ERR_FAIL;
    }

    if (!fs_exists(path)) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }
    if (fs_remove(path)) {
        return ServerResult::SUCCESS;
    } else {
        return ServerResult::ERR_FAIL;
    }
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_rename(PayloadHeader* payload)
{
    size_t term_i = payload->size;
    if (payload->size >= max_data_length) {
        term_i = max_data_length - 1;
    }
    payload->data[term_i] = '\0';

    std::string old_name = std::string(reinterpret_cast<char*>(&(payload->data[0])));
    std::string new_name =
        _get_path(std::string(reinterpret_cast<char*>(&(payload->data[old_name.length() + 1]))));
    old_name = _get_path(old_name);
    if (old_name.rfind(_root_dir, 0) != 0 || new_name.rfind(_root_dir, 0) != 0) {
        return ServerResult::ERR_FAIL;
    }

    if (!fs_exists(old_name)) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    if (fs_rename(old_name, new_name)) {
        return ServerResult::SUCCESS;
    } else {
        return ServerResult::ERR_FAIL;
    }
}

MavlinkFtpServer::ServerResult
MavlinkFtpServer::_calc_local_file_crc32(const std::string& path, uint32_t& csum)
{
    if (!fs_exists(path)) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        return ServerResult::ERR_FILE_IO_ERROR;
    }

    // Read whole file in buffer size chunks
    Crc32 checksum;
    char buffer[18392];
    ssize_t bytes_read;
    do {
        bytes_read = ::read(fd, buffer, sizeof(buffer));

        if (bytes_read < 0) {
            int r_errno = errno;
            close(fd);
            errno = r_errno;
            return ServerResult::ERR_FILE_IO_ERROR;
        }

        checksum.add((uint8_t*)buffer, bytes_read);
    } while (bytes_read == sizeof(buffer));

    close(fd);

    csum = checksum.get();

    return ServerResult::SUCCESS;
}

MavlinkFtpServer::ServerResult MavlinkFtpServer::_work_calc_file_CRC32(PayloadHeader* payload)
{
    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        return ServerResult::ERR_FAIL;
    }

    if (!fs_exists(path)) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    payload->size = sizeof(uint32_t);
    uint32_t checksum;
    ServerResult res = _calc_local_file_crc32(path, checksum);
    if (res != ServerResult::SUCCESS) {
        return res;
    }
    *reinterpret_cast<uint32_t*>(payload->data) = checksum;

    return ServerResult::SUCCESS;
}

void MavlinkFtpServer::send()
{
    // Anything to stream?
    if (!_session_info.stream_download) {
        return;
    }
}

std::optional<std::string>
MavlinkFtpServer::write_tmp_file(const std::string& path, const std::string& content)
{
    // TODO: Check if currently an operation is ongoing.

    if (path.find("..") != std::string::npos) {
        LogWarn() << "Path with .. not supported.";
        return {};
    }

    if (path.find('/') != std::string::npos) {
        LogWarn() << "Path with / not supported.";
        return {};
    }

    if (path.find('\\') != std::string::npos) {
        LogWarn() << "Path with \\ not supported.";
        return {};
    }

    // We use a temporary directory to put these
    if (_tmp_dir.empty()) {
        auto maybe_tmp_dir = create_tmp_directory("mavsdk-mavlink-ftp-tmp-files");
        if (maybe_tmp_dir) {
            _tmp_dir = maybe_tmp_dir.value();
        }
        // If we can't get a tmp dir, we'll just try to use our current working dir,
        // or whatever is the root dir by default.
    }

    const auto file_path = _tmp_dir + path_separator + path;
    std::ofstream out(file_path);
    out << content;
    out.flush();
    if (out.bad()) {
        LogWarn() << "Writing to " << file_path << " failed";
        out.close();
        return {};
    } else {
        out.close();

        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        _tmp_files[path] = file_path;

        return {file_path};
    }
}

} // namespace mavsdk
