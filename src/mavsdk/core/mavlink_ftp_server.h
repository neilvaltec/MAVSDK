#pragma once

#include <cinttypes>
#include <functional>
#include <fstream>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mavlink_include.h"

// As found in
// https://stackoverflow.com/questions/1537964#answer-3312896
#ifdef _MSC_VER // MSVC
#define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#else
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#endif

namespace mavsdk {

class ServerComponentImpl;

class MavlinkFtpServer {
public:
    explicit MavlinkFtpServer(ServerComponentImpl& server_component_impl);
    ~MavlinkFtpServer();

    struct ProgressData {
        uint32_t bytes_transferred{}; /**< @brief The number of bytes already transferred. */
        uint32_t total_bytes{}; /**< @brief The total bytes to transfer. */
    };

    void send();

    void set_retries(uint32_t retries) { _max_last_command_retries = retries; }
    void set_root_directory(const std::string& root_dir);

    std::optional<std::string> write_tmp_file(const std::string& path, const std::string& content);

private:
    ServerComponentImpl& _server_component_impl;

    /// @brief Possible server results returned for requests.
    enum ServerResult : uint8_t {
        SUCCESS,
        ERR_FAIL, ///< Unknown failure
        ERR_FAIL_ERRNO, ///< Command failed, errno sent back in PayloadHeader.data[1]
        ERR_INVALID_DATA_SIZE, ///< PayloadHeader.size is invalid
        ERR_INVALID_SESSION, ///< Session is not currently open
        ERR_NO_SESSIONS_AVAILABLE, ///< All available Sessions in use
        ERR_EOF, ///< Offset past end of file for List and Read commands
        ERR_UNKOWN_COMMAND, ///< Unknown command opcode
        ERR_FAIL_FILE_EXISTS, ///< File exists already
        ERR_FAIL_FILE_PROTECTED, ///< File is write protected
        ERR_FAIL_FILE_DOES_NOT_EXIST, ///< File does not exist

        // These error codes are returned to client without contacting the server
        ERR_TIMEOUT = 200, ///< Timeout
        ERR_FILE_IO_ERROR, ///< File IO operation error
    };

    /// @brief Command opcodes
    enum class Opcode : uint8_t {
        CMD_NONE, ///< ignored, always acked
        CMD_TERMINATE_SESSION, ///< Terminates open Read session
        CMD_RESET_SESSIONS, ///< Terminates all open Read sessions
        CMD_LIST_DIRECTORY, ///< List files in <path> from <offset>
        CMD_OPEN_FILE_RO, ///< Opens file at <path> for reading, returns <session>
        CMD_READ_FILE, ///< Reads <size> bytes from <offset> in <session>
        CMD_CREATE_FILE, ///< Creates file at <path> for writing, returns <session>
        CMD_WRITE_FILE, ///< Writes <size> bytes to <offset> in <session>
        CMD_REMOVE_FILE, ///< Remove file at <path>
        CMD_CREATE_DIRECTORY, ///< Creates directory at <path>
        CMD_REMOVE_DIRECTORY, ///< Removes Directory at <path>, must be empty
        CMD_OPEN_FILE_WO, ///< Opens file at <path> for writing, returns <session>
        CMD_TRUNCATE_FILE, ///< Truncate file at <path> to <offset> length
        CMD_RENAME, ///< Rename <path1> to <path2>
        CMD_CALC_FILE_CRC32, ///< Calculate CRC32 for file at <path>
        CMD_BURST_READ_FILE, ///< Burst download session file

        RSP_ACK = 128, ///< Ack response
        RSP_NAK ///< Nak response
    };

    /// @brief Maximum data size in RequestHeader::data
    static constexpr uint8_t max_data_length = 239;

    /// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
    /// This needs to be packed, because it's typecasted from
    /// mavlink_file_transfer_protocol_t.payload, which starts at a 3 byte offset, causing an
    /// unaligned access to seq_number and offset
    PACK(struct PayloadHeader {
        uint16_t seq_number; ///< sequence number for message
        uint8_t session; ///< Session id for read and write commands
        Opcode opcode; ///< Command opcode
        uint8_t size; ///< Size of data
        Opcode req_opcode; ///< Request opcode returned in RSP_ACK, RSP_NAK message
        uint8_t burst_complete; ///< Only used if req_opcode=CMD_BURST_READ_FILE - 1: set of burst
        ///< packets complete, 0: More burst packets coming.
        uint8_t padding; ///< 32 bit alignment padding
        uint32_t offset; ///< Offsets for List and Read commands
        uint8_t data[max_data_length]; ///< command data, varies by Opcode
    });

    static_assert(
        sizeof(PayloadHeader) == sizeof(mavlink_file_transfer_protocol_t::payload),
        "PayloadHeader size is incorrect.");

    struct SessionInfo {
        int fd{-1};
        uint32_t file_size{0};
        bool stream_download{false};
        uint32_t stream_offset{0};
        uint16_t stream_seq_number{0};
        uint8_t stream_target_system_id{0};
        unsigned stream_chunk_transmitted{0};
    };

    struct OfstreamWithPath {
        std::ofstream stream;
        std::string path;
    };

    struct SessionInfo _session_info {}; ///< Session info, fd=-1 for no active session

    uint8_t _network_id = 0;
    uint8_t _target_system_id = 0;
    uint8_t _target_component_id = 0;
    Opcode _curr_op = Opcode::CMD_NONE;
    std::mutex _curr_op_mutex{};
    mavlink_message_t _last_command{};
    uint32_t _max_last_command_retries{5};
    uint32_t _last_command_retries = 0;
    std::string _last_path{};
    uint16_t _seq_number = 0;
    std::ifstream _ifstream{};
    OfstreamWithPath _ofstream{};
    bool _session_valid = false;
    uint8_t _session = 0;
    ServerResult _session_result = ServerResult::SUCCESS;
    uint32_t _bytes_transferred = 0;
    uint32_t _file_size = 0;
    std::vector<std::string> _curr_directory_list{};

    ServerResult _calc_local_file_crc32(const std::string& path, uint32_t& csum);

    void _call_op_result_callback(ServerResult result);
    void _call_op_progress_callback(uint32_t bytes_written, uint32_t total_bytes);
    void _call_dir_items_result_callback(ServerResult result, std::vector<std::string> list);
    void _call_crc32_result_callback(ServerResult result, uint32_t crc32);
    void _write();
    void _end_read_session(bool delete_file = false);
    void _terminate_session();
    void _send_mavlink_ftp_message(const PayloadHeader& payload);

    void _list_directory(uint32_t offset);

    void _reset();

    // prepend a root directory to each file/dir access to avoid enumerating the full FS tree
    std::string _root_dir{};

    bool _last_reply_valid = false;
    mavlink_message_t _last_reply{};

    void process_mavlink_ftp_message(const mavlink_message_t& msg);

    std::string _data_as_string(const PayloadHeader& payload);
    std::string _get_path(const PayloadHeader& payload);
    std::string _get_path(const std::string& payload_path);

    void _work_list(const PayloadHeader& payload, PayloadHeader& response);
    void _work_open_file_readonly(const PayloadHeader& payload, PayloadHeader& response);
    void _work_open_file_writeonly(const PayloadHeader& payload, PayloadHeader& response);
    void _work_create_file(const PayloadHeader& payload, PayloadHeader& response);
    void _work_read(const PayloadHeader& payload, PayloadHeader& response);
    void _work_burst(const PayloadHeader& payload, PayloadHeader& response);
    void _work_write(const PayloadHeader& payload, PayloadHeader& response);
    void _work_terminate(const PayloadHeader& payload, PayloadHeader& response);
    void _work_reset(const PayloadHeader& payload, PayloadHeader& response);
    void _work_remove_directory(const PayloadHeader& payload, PayloadHeader& response);
    void _work_create_directory(const PayloadHeader& payload, PayloadHeader& response);
    void _work_remove_file(const PayloadHeader& payload, PayloadHeader& response);
    void _work_rename(const PayloadHeader& payload, PayloadHeader& response);
    void _work_calc_file_CRC32(const PayloadHeader& payload, PayloadHeader& response);

    std::mutex _tmp_files_mutex{};
    std::unordered_map<std::string, std::string> _tmp_files{};
    std::string _tmp_dir{};

    bool _debugging{false};
};

} // namespace mavsdk
