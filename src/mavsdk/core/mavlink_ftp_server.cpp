#include <algorithm>
#include <fstream>
#include <filesystem>
#include <future>

#include "mavlink_ftp_server.h"
#include "server_component_impl.h"
#include "unused.h"
#include "crc32.h"

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

std::string MavlinkFtpServer::_data_as_string(const PayloadHeader& payload, size_t entry)
{
    size_t start = 0;
    size_t end = 0;
    std::string result;

    for (int i = entry; i >= 0; --i) {
        start = end;
        end +=
            strnlen(reinterpret_cast<const char*>(&payload.data[start]), max_data_length - start) +
            1;
    }

    result.resize(end - start);
    std::memcpy(result.data(), &payload.data[start], end - start);

    return result;
}

std::variant<std::string, MavlinkFtpServer::ServerResult>
MavlinkFtpServer::_path_from_payload(const PayloadHeader& payload, size_t entry)
{
    auto data = _data_as_string(payload, entry);
    return _path_from_string(data);
}

std::variant<std::string, MavlinkFtpServer::ServerResult>
MavlinkFtpServer::_path_from_string(const std::string& payload_path)
{
    // No permission whatsoever if the root dir is not set.
    if (_root_dir.empty()) {
        return ServerResult::ERR_FAIL;
    }

    fs::path combined_path = (fs::path(_root_dir) / payload_path).lexically_normal();

    // Check whether the combined path is inside the root dir.
    // From: https://stackoverflow.com/a/61125335/8548472
    auto ret = std::mismatch(_root_dir.begin(), _root_dir.end(), combined_path.string().begin());
    if (ret.first != _root_dir.end()) {
        LogWarn() << "Not inside root dir";
        return ServerResult::ERR_FAIL;
    }

    return combined_path.string();
}

void MavlinkFtpServer::set_root_directory(const std::string& root_dir)
{
    std::error_code ignored;
    _root_dir = fs::canonical(fs::path(root_dir), ignored).string();
}

void MavlinkFtpServer::_work_list(const PayloadHeader& payload, PayloadHeader& response)
{
    auto maybe_path = _path_from_payload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        return;
    }

    fs::path path = std::get<std::string>(maybe_path);

    uint8_t offset = 0;

    // Move to the requested offset
    uint32_t requested_offset = payload.offset;

    std::error_code ec;
    if (!fs::exists(path, ec)) {
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
    if (_session_info.ifstream.is_open()) {
        _reset();
    }

    std::string path;
    {
        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        const auto it = _tmp_files.find(_data_as_string(payload));
        if (it != _tmp_files.end()) {
            path = it->second;
        } else {
            auto maybe_path = _path_from_payload(payload);
            if (std::holds_alternative<ServerResult>(maybe_path)) {
                response.opcode = Opcode::RSP_NAK;
                response.size = 1;
                response.data[0] = std::get<ServerResult>(maybe_path);
                return;
            }

            path = std::get<std::string>(maybe_path);
        }
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

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        LogErr() << "FTP: Open failed - file doesn't exist";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    auto file_size = static_cast<uint32_t>(fs::file_size(path, ec));
    if (ec) {
        LogErr() << "Could not determine file size of '" << path << "': " << ec.message();
        return;
    }

    if (_debugging) {
        LogDebug() << "Determined filesize to be: " << file_size << " bytes";
    }

    std::ifstream ifstream;
    ifstream.open(path, std::ios::in | std::ios::binary);

    if (!ifstream.is_open()) {
        LogWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    _session_info.ifstream = std::move(ifstream);
    _session_info.file_size = file_size;
    _session_info.stream_download = false;

    response.opcode = Opcode::RSP_ACK;
    response.session = 0;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &file_size, response.size);
}

void MavlinkFtpServer::_work_open_file_writeonly(
    const PayloadHeader& payload, PayloadHeader& response)
{
    if (_session_info.ofstream.is_open()) {
        _reset();
    }

    std::string path;
    {
        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        const auto it = _tmp_files.find(_data_as_string(payload));
        if (it != _tmp_files.end()) {
            path = it->second;
        } else {
            auto maybe_path = _path_from_payload(payload);
            if (std::holds_alternative<ServerResult>(maybe_path)) {
                response.opcode = Opcode::RSP_NAK;
                response.size = 1;
                response.data[0] = std::get<ServerResult>(maybe_path);
                return;
            }

            path = std::get<std::string>(maybe_path);
        }
    }

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
    std::error_code ec;
    if (!fs::exists(path, ec)) {
        LogWarn() << "FTP: Open failed - file not found";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    auto file_size = static_cast<uint32_t>(fs::file_size(path, ec));
    if (ec) {
        LogErr() << "Could not determine file size of '" << path << "': " << ec.message();
        return;
    }

    if (_debugging) {
        LogDebug() << "Determined filesize to be: " << file_size << " bytes";
    }

    std::ofstream ofstream;
    ofstream.open(path, std::ios::out | std::ios::binary);

    if (!ofstream.is_open()) {
        LogWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    _session_info.ofstream = std::move(ofstream);
    _session_info.file_size = file_size;
    _session_info.stream_download = false;

    response.opcode = Opcode::RSP_ACK;
    response.session = 0;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &file_size, response.size);
}

void MavlinkFtpServer::_work_create_file(const PayloadHeader& payload, PayloadHeader& response)
{
    if (_session_info.ofstream.is_open()) {
        _reset();
    }

    std::string path;
    {
        std::lock_guard<std::mutex> lock(_tmp_files_mutex);
        const auto it = _tmp_files.find(_data_as_string(payload));
        if (it != _tmp_files.end()) {
            path = it->second;
        } else {
            auto maybe_path = _path_from_payload(payload);
            if (std::holds_alternative<ServerResult>(maybe_path)) {
                response.opcode = Opcode::RSP_NAK;
                response.size = 1;
                response.data[0] = std::get<ServerResult>(maybe_path);
                return;
            }

            path = std::get<std::string>(maybe_path);
        }
    }

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

    std::ofstream ofstream;
    ofstream.open(path, std::ios::out | std::ios::binary);

    if (!ofstream.is_open()) {
        LogWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        return;
    }

    _session_info.ofstream = std::move(ofstream);
    _session_info.stream_download = false;
    _session_info.file_size = 0;

    response.session = 0;
    response.size = 0;
    response.opcode = Opcode::RSP_ACK;
}

void MavlinkFtpServer::_work_read(const PayloadHeader& payload, PayloadHeader& response)
{
    if (payload.session != 0 || !_session_info.ifstream.is_open()) {
        _reset();
    }

    // We have to test seek past EOF ourselves, lseek will allow seek past EOF
    if (payload.offset >= _session_info.file_size) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_EOF;
        if (_debugging) {
            LogDebug() << "Reached EOF reading";
        }
        return;
    }

    _session_info.ifstream.seekg(payload.offset);
    if (_session_info.ifstream.fail()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        LogWarn() << "Seek failed";
        return;
    }

    _session_info.ifstream.read(reinterpret_cast<char*>(response.data), payload.size);

    if (_session_info.ifstream.fail()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        LogWarn() << "Read failed";
        return;
    }

    const uint32_t bytes_read = _session_info.ifstream.gcount();

    response.size = bytes_read;
    response.opcode = Opcode::RSP_ACK;
}

void MavlinkFtpServer::_work_burst(const PayloadHeader& payload, PayloadHeader& response)
{
    // TODO: implement burst
    UNUSED(payload);
    UNUSED(response);

    // if (payload.session != 0 && _session_info.fd < 0) {
    //    _reset();
    //}

    //// Setup for streaming sends
    //_session_info.stream_download = true;
    //_session_info.stream_offset = payload.offset;
    //_session_info.stream_chunk_transmitted = 0;
    //_session_info.stream_seq_number = payload.seq_number + 1;
    //_session_info.stream_target_system_id = 0; // FIXME: _server_component_impl.get_system_id();
}

void MavlinkFtpServer::_work_write(const PayloadHeader& payload, PayloadHeader& response)
{
    if (payload.session != 0 && !_session_info.ofstream.is_open()) {
        _reset();
    }

    _session_info.ofstream.seekp(payload.offset);
    if (_session_info.ifstream.fail()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        LogWarn() << "Seek failed";
        return;
    }

    _session_info.ofstream.write(reinterpret_cast<const char*>(payload.data), payload.size);
    if (_session_info.ofstream.fail()) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        LogWarn() << "Write failed";
        return;
    }

    response.opcode = Opcode::RSP_ACK;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &payload.size, response.size);
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
    if (_session_info.ifstream.is_open()) {
        _session_info.ifstream.close();
    }

    if (_session_info.ofstream.is_open()) {
        _session_info.ofstream.close();
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
    auto maybe_path = _path_from_payload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        return;
    }

    fs::path path = std::get<std::string>(maybe_path);

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }
    if (fs::remove(path, ec)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
    }
}

void MavlinkFtpServer::_work_create_directory(const PayloadHeader& payload, PayloadHeader& response)
{
    auto maybe_path = _path_from_payload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        return;
    }

    auto path = std::get<std::string>(maybe_path);

    std::error_code ec;
    if (fs::exists(path, ec)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_EXISTS;
        return;
    }

    if (fs::create_directory(path, ec)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 2;
        response.data[0] = ServerResult::ERR_FAIL_ERRNO;
        response.data[1] = static_cast<uint8_t>(ec.value());
    }
}

void MavlinkFtpServer::_work_remove_file(const PayloadHeader& payload, PayloadHeader& response)
{
    auto maybe_path = _path_from_payload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        return;
    }

    auto path = std::get<std::string>(maybe_path);

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }
    if (fs::remove(path, ec)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
    }
}

void MavlinkFtpServer::_work_rename(const PayloadHeader& payload, PayloadHeader& response)
{
    auto maybe_old_name = _path_from_payload(payload, 0);

    if (std::holds_alternative<ServerResult>(maybe_old_name)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_old_name);
        return;
    }

    auto old_name = std::get<std::string>(maybe_old_name);

    auto maybe_new_name = _path_from_payload(payload, 1);

    if (std::holds_alternative<ServerResult>(maybe_new_name)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_new_name);
        return;
    }

    auto new_name = std::get<std::string>(maybe_new_name);

    if (_debugging) {
        LogDebug() << "Rename from old_name " << old_name << " to " << new_name;
    }

    std::error_code ec;
    if (!fs::exists(old_name, ec)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        return;
    }

    fs::rename(old_name, new_name, ec);
    if (!ec) {
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
    std::error_code ec;
    if (!fs::exists(path, ec)) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    std::ifstream ifstream;
    ifstream.open(path, std::ios::in | std::ios::binary);

    if (!ifstream.is_open()) {
        return ServerResult::ERR_FILE_IO_ERROR;
    }

    // Read whole file in buffer size chunks
    Crc32 checksum;
    char buffer[18392];
    do {
        ifstream.read(buffer, sizeof(buffer));

        if (ifstream.fail() && !ifstream.eof()) {
            ifstream.close();
            return ServerResult::ERR_FILE_IO_ERROR;
        }

        auto bytes_read = ifstream.gcount();
        checksum.add((uint8_t*)buffer, bytes_read);

    } while (!ifstream.eof());

    ifstream.close();

    csum = checksum.get();

    return ServerResult::SUCCESS;
}

void MavlinkFtpServer::_work_calc_file_CRC32(const PayloadHeader& payload, PayloadHeader& response)
{
    auto maybe_path = _path_from_payload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        return;
    }

    auto path = std::get<std::string>(maybe_path);

    std::error_code ec;
    if (!fs::exists(path, ec)) {
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

} // namespace mavsdk
