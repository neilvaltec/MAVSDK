#include "mavlink_ftp_server.h"
#include "server_component_impl.h"
#include <fstream>
#include <filesystem>

#if defined(WINDOWS)
#include "tronkko_dirent.h"
#include "stackoverflow_unistd.h"
#else
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

namespace fs = std::filesystem;

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
    // TODO: implement stream sending
    bool stream_send = false;
    UNUSED(stream_send);

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

    const PayloadHeader& payload = *reinterpret_cast<PayloadHeader*>(&ftp_req.payload[0]);

    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;

    // Basic sanity check: validate length before use.
    if (payload.size > max_data_length) {
        response.opcode = Opcode::RSP_NAK;
        response.data[0] = ServerResult::ERR_INVALID_DATA_SIZE;
        response.size = 1;

    } else {
        if (_debugging) {
            LogDebug() << "FTP opcode: " << (int)payload.opcode << ", size: " << (int)payload.size
                       << ", offset: " << (int)payload.offset << ", seq: " << payload.seq_number;
        }

        if (_curr_op != Opcode::CMD_NONE && _curr_op != payload.req_opcode) {
            LogWarn() << "Received ACK not matching our current operation, resetting";
            _reset();
        }

        _target_system_id = msg.sysid;
        _target_component_id = msg.compid;

        switch (payload.opcode) {
            case Opcode::CMD_NONE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_NONE";
                }
                break;

            case Opcode::CMD_TERMINATE_SESSION:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_TERMINATE_SESSION";
                }
                _work_terminate(payload, response);
                break;

            case Opcode::CMD_RESET_SESSIONS:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_RESET_SESSIONS";
                }
                _work_reset(payload, response);
                break;

            case Opcode::CMD_LIST_DIRECTORY:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_LIST_DIRECTORY";
                }
                _work_list(payload, response);
                break;

            case Opcode::CMD_OPEN_FILE_RO:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_OPEN_FILE_RO";
                }
                _work_open_file_readonly(payload, response);
                break;

            case Opcode::CMD_CREATE_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_CREATE_FILE";
                }
                _work_create_file(payload, response);
                break;

            case Opcode::CMD_OPEN_FILE_WO:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_OPEN_FILE_WO";
                }
                _work_open_file_writeonly(payload, response);
                break;

            case Opcode::CMD_READ_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_READ_FILE";
                }
                _work_read(payload, response);
                break;

            case Opcode::CMD_BURST_READ_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_BURST_READ_FILE";
                }
                _work_burst(payload, response);
                stream_send = true;
                break;

            case Opcode::CMD_WRITE_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_WRITE_FILE";
                }
                _work_write(payload, response);
                break;

            case Opcode::CMD_REMOVE_FILE:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_REMOVE_FILE";
                }
                _work_remove_file(payload, response);
                break;

            case Opcode::CMD_RENAME:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_RENAME";
                }
                _work_rename(payload, response);
                break;

            case Opcode::CMD_CREATE_DIRECTORY:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_CREATE_DIRECTORY";
                }
                _work_create_directory(payload, response);
                break;

            case Opcode::CMD_REMOVE_DIRECTORY:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_REMOVE_DIRECTORY";
                }
                _work_remove_directory(payload, response);
                break;

            case Opcode::CMD_CALC_FILE_CRC32:
                if (_debugging) {
                    LogDebug() << "OPC:CMD_CALC_FILE_CRC32";
                }
                _work_calc_file_CRC32(payload, response);
                break;

            default:
                // Not for us, ignore it.
                return;
        }
    }

    // handle success vs. error
    // if (error_code == ServerResult::SUCCESS) {
    //    payload->req_opcode = payload->opcode;
    //    payload->opcode = RSP_ACK;

    //} else {
    //    uint8_t r_errno = errno;
    //    payload->req_opcode = payload->opcode;
    //    payload->opcode = RSP_NAK;
    //    payload->size = 1;

    //    if (r_errno == EEXIST) {
    //        error_code = ServerResult::ERR_FAIL_FILE_EXISTS;
    //    } else if (r_errno == ENOENT && error_code == ServerResult::ERR_FAIL_ERRNO) {
    //        error_code = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    //    }

    //    *reinterpret_cast<ServerResult*>(payload->data) = error_code;

    //    if (error_code == ServerResult::ERR_FAIL_ERRNO) {
    //        payload->size = 2;
    //        *reinterpret_cast<uint8_t*>(payload->data + 1) = r_errno;
    //    }
    //}

    //_last_reply_valid = false;

    // Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
    // if (!stream_send || error_code != ServerResult::SUCCESS) {
    //    // keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS
    //    // resends the request, we can simply resend the response.
    //    _last_reply_valid = true;

    mavlink_msg_file_transfer_protocol_pack(
        _server_component_impl.get_own_system_id(),
        _server_component_impl.get_own_component_id(),
        &_last_reply,
        _network_id,
        _target_system_id,
        _target_component_id,
        reinterpret_cast<const uint8_t*>(&response));
    _server_component_impl.send_message(_last_reply);
    //}
}

MavlinkFtpServer::~MavlinkFtpServer()
{
    _reset();
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

std::string MavlinkFtpServer::_data_as_string(const PayloadHeader& payload)
{
    std::string result;

    // Guarantee null termination
    if (payload.size < max_data_length) {
        result.resize(payload.size + 1);
        std::memcpy(result.data(), payload.data, payload.size);

    } else {
        // This covers the case where there is no space for null termination .
        result.resize(max_data_length);
        std::memcpy(result.data(), payload.data, max_data_length);
    }

    return result;
}

std::string MavlinkFtpServer::_get_path(const PayloadHeader& payload)
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

void MavlinkFtpServer::_work_list(const PayloadHeader& payload, PayloadHeader& response)
{
    if (_root_dir.empty()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    uint8_t offset = 0;

    // Move to the requested offset
    uint32_t requested_offset = payload.offset;

    fs::path path = _get_path(payload);
    if (path.string().rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path.string();
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_PROTECTED;
        return;
    }
    if (!fs_exists(path)) {
        LogWarn() << "FTP: can't open path " << path;
        // this is not an FTP error, abort directory by simulating eof
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    if (_debugging) {
        LogDebug() << "Opening path: " << path.string();
    }

    for (const auto& entry : fs::directory_iterator(fs::canonical(path))) {
        if (requested_offset > 0) {
            requested_offset--;
            continue;
        }
        const auto name = entry.path().filename();

        std::string payload_str;
        std::error_code ec;

        const auto is_regular_file = entry.is_regular_file(ec);
        if (ec) {
            LogWarn() << "Could not determine whether '" << entry.path().string()
                      << "' is a file: " << ec.message();
            continue;
        }

        const auto is_directory = entry.is_directory(ec);
        if (ec) {
            LogWarn() << "Could not determine whether '" << entry.path().string()
                      << "' is a directory: " << ec.message();
            continue;
        }

        if (is_regular_file) {
            const auto filesize = fs::file_size(entry.path(), ec);
            if (ec) {
                LogWarn() << "Could not get file size of '" << entry.path().string()
                          << "': " << ec.message();
                continue;
            }

            if (_debugging) {
                LogDebug() << "Found file: " << name.string() << ", size: " << filesize << " bytes";
            }

            payload_str += 'F';
            payload_str += name.string();
            payload_str += '\t';
            payload_str += std::to_string(filesize);

        } else if (is_directory) {
            if (_debugging) {
                LogDebug() << "Found directory: " << name.string();
            }

            payload_str += 'D';
            payload_str += name.string();

        } else {
            // Ignore all other types.
            continue;
        }

        auto required_len = payload_str.length() + 1;

        // Do we have room for the dir entry and the null terminator?
        if (offset + required_len > max_data_length) {
            break;
        }

        std::memcpy(&response.data[offset], payload_str.c_str(), required_len);

        offset += static_cast<uint8_t>(required_len);
    }

    if (offset == 0) {
        // We are done and need to respond with EOF.
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_EOF;

    } else {
        response.opcode = Opcode::RSP_ACK;
        response.size = offset;
    }
}

void MavlinkFtpServer::_work_open_file_readonly(
    const PayloadHeader& payload, PayloadHeader& response)
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
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    if (_debugging) {
        LogInfo() << "Finding " << path << " in " << _root_dir;
    }
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (_debugging) {
        LogDebug() << "Going to open readonly: " << path;
    }

    if (!fs_exists(path)) {
        LogWarn() << "FTP: Open failed - file not found";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    uint32_t file_size = fs_file_size(path);

    if (_debugging) {
        LogDebug() << "Determined filesize to be: " << file_size << " bytes";
    }

    int fd = ::open(path.c_str(), O_RDONLY);

    if (fd < 0) {
        LogWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    _session_info.fd = fd;
    _session_info.file_size = file_size;
    _session_info.stream_download = false;

    response.opcode = Opcode::RSP_ACK;
    response.session = 0;
    response.size = sizeof(uint32_t);
    memcpy(response.data, &file_size, response.size);
}

void MavlinkFtpServer::_work_open_file_writeonly(
    const PayloadHeader& payload, PayloadHeader& response)
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
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    if (_debugging) {
        LogInfo() << "Finding " << path << " in " << _root_dir;
    }
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (_debugging) {
        LogDebug() << "Going to open writeonly: " << path;
    }

    // fail only if requested open for read
    if (!fs_exists(path)) {
        LogWarn() << "FTP: Open failed - file not found";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    uint32_t file_size = fs_file_size(path);

    int fd = ::open(path.c_str(), O_WRONLY, 0666);

    if (fd < 0) {
        LogWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] =
            (errno == ENOENT) ? ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST : ServerResult::ERR_FAIL;
        return;
    }

    _session_info.fd = fd;
    _session_info.file_size = file_size;
    _session_info.stream_download = false;

    response.opcode = Opcode::RSP_ACK;
    response.session = 0;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &file_size, response.size);
}

void MavlinkFtpServer::_work_create_file(const PayloadHeader& payload, PayloadHeader& response)
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
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    if (_debugging) {
        LogInfo() << "Finding " << path << " in " << _root_dir;
    }
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (_debugging) {
        LogDebug() << "Creating file: " << path;
    }

    int fd = ::open(path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0666);

    if (fd < 0) {
        LogWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    _session_info.fd = fd;
    _session_info.file_size = 0;

    response.session = 0;
    response.size = 0;
    response.opcode = Opcode::RSP_ACK;
}

void MavlinkFtpServer::_work_read(const PayloadHeader& payload, PayloadHeader& response)
{
    if (payload.session != 0 || _session_info.fd < 0) {
        _reset();
    }

    // We have to test seek past EOF ourselves, lseek will allow seek past EOF
    if (payload.offset >= _session_info.file_size) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_EOF;
        return;
    }

    if (lseek(_session_info.fd, payload.offset, SEEK_SET) < 0) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    auto bytes_read = ::read(_session_info.fd, &response.data[0], max_data_length);

    if (bytes_read < 0) {
        // Negative return indicates error other than EOF.
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    response.size = bytes_read;
    response.opcode = Opcode::RSP_ACK;
}

void MavlinkFtpServer::_work_burst(const PayloadHeader& payload, PayloadHeader& response)
{
    // TODO: implement burst
    UNUSED(response);

    if (payload.session != 0 && _session_info.fd < 0) {
        _reset();
    }

    // Setup for streaming sends
    _session_info.stream_download = true;
    _session_info.stream_offset = payload.offset;
    _session_info.stream_chunk_transmitted = 0;
    _session_info.stream_seq_number = payload.seq_number + 1;
    _session_info.stream_target_system_id = 0; // FIXME: _server_component_impl.get_system_id();
}

void MavlinkFtpServer::_work_write(const PayloadHeader& payload, PayloadHeader& response)
{
    if (payload.session != 0 && _session_info.fd < 0) {
        _reset();
    }

    if (lseek(_session_info.fd, payload.offset, SEEK_SET) < 0) {
        // Unable to see to the specified location
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    const int bytes_written = ::write(_session_info.fd, &payload.data[0], payload.size);

    if (bytes_written < 0) {
        // Negative return indicates error other than eof
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    response.opcode = Opcode::RSP_ACK;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &bytes_written, response.size);
}

void MavlinkFtpServer::_work_terminate(const PayloadHeader& payload, PayloadHeader& response)
{
    UNUSED(payload);
    UNUSED(response);
    _reset();

    response.opcode = Opcode::RSP_ACK;
    response.size = 0;
}

void MavlinkFtpServer::_reset()
{
    if (_session_info.fd != -1) {
        close(_session_info.fd);
        _session_info.fd = -1;
        _session_info.stream_download = false;
    }
}

void MavlinkFtpServer::_work_reset(const PayloadHeader& payload, PayloadHeader& response)
{
    UNUSED(payload);
    UNUSED(response);
    _reset();
}

void MavlinkFtpServer::_work_remove_directory(const PayloadHeader& payload, PayloadHeader& response)
{
    if (_root_dir.empty()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (!fs_exists(path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }
    if (fs_remove(path)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
    }
}

void MavlinkFtpServer::_work_create_directory(const PayloadHeader& payload, PayloadHeader& response)
{
    if (_root_dir.empty()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (fs_exists(path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_EXISTS;
        return;
    }
    if (fs_create_directory(path)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 2;
        response.data[0] = ServerResult::ERR_FAIL_ERRNO;
        response.data[1] = errno;
    }
}

void MavlinkFtpServer::_work_remove_file(const PayloadHeader& payload, PayloadHeader& response)
{
    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (!fs_exists(path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }
    if (fs_remove(path)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
    }
}

void MavlinkFtpServer::_work_rename(const PayloadHeader& payload, PayloadHeader& response)
{
    size_t term_i = payload.size;
    if (payload.size >= max_data_length) {
        term_i = max_data_length - 1;
    }
    response.data[term_i] = '\0';

    std::string old_name = std::string(reinterpret_cast<const char*>(&(payload.data[0])));
    std::string new_name = _get_path(
        std::string(reinterpret_cast<const char*>(&(payload.data[old_name.length() + 1]))));
    old_name = _get_path(old_name);
    if (old_name.rfind(_root_dir, 0) != 0 || new_name.rfind(_root_dir, 0) != 0) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (!fs_exists(old_name)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    if (fs_rename(old_name, new_name)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
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

void MavlinkFtpServer::_work_calc_file_CRC32(const PayloadHeader& payload, PayloadHeader& response)
{
    std::string path = _get_path(payload);
    if (path.rfind(_root_dir, 0) != 0) {
        LogWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    if (!fs_exists(path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    uint32_t checksum;
    ServerResult res = _calc_local_file_crc32(path, checksum);
    if (res != ServerResult::SUCCESS) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = res;
        return;
    }

    response.opcode = Opcode::RSP_ACK;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &checksum, response.size);
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
