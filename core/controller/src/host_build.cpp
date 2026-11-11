#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <array>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"


namespace mechsim::controller {

namespace {

namespace fs = std::filesystem;

struct ProcessResult {
    bool succeeded = false;
    std::string output;
};

std::string command_line(const std::vector<std::string>& arguments) {
    std::string line;
    for (const std::string& argument : arguments) {
        if (!line.empty()) {
            line += ' ';
        }
        line += argument;
    }
    return line;
}

#if defined(_WIN32)
std::wstring widen(const std::string& text) {
    if (text.empty()) {
        return {};
    }
    const int size = ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    std::wstring wide(static_cast<std::size_t>(size), L'\0');
    ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), wide.data(), size);
    return wide;
}

std::string narrow(const std::wstring& wide) {
    if (wide.empty()) {
        return {};
    }
    const int size =
        ::WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), nullptr, 0, nullptr, nullptr);
    std::string text(static_cast<std::size_t>(size), '\0');
    ::WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), text.data(), size, nullptr,
                          nullptr);
    return text;
}

std::string error_text(DWORD code) {
    LPWSTR buffer = nullptr;
    const DWORD length = ::FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code, 0,
        reinterpret_cast<LPWSTR>(&buffer), 0, nullptr);
    std::string text = "error " + std::to_string(code);
    if (length > 0 && buffer != nullptr) {
        std::wstring message(buffer, length);
        while (!message.empty() && (message.back() == L'\n' || message.back() == L'\r' || message.back() == L' ')) {
            message.pop_back();
        }
        text = narrow(message);
    }
    if (buffer != nullptr) {
        ::LocalFree(buffer);
    }
    return text;
}

// CommandLineToArgvW quoting: backslashes double only before a quote or the closing quote.
std::wstring quote_argument(const std::wstring& argument) {
    if (!argument.empty() && argument.find_first_of(L" \t\n\v\"") == std::wstring::npos) {
        return argument;
    }
    std::wstring quoted = L"\"";
    std::size_t backslashes = 0;
    for (const wchar_t character : argument) {
        if (character == L'\\') {
            ++backslashes;
        } else if (character == L'"') {
            quoted.append((backslashes * 2) + 1, L'\\');
            quoted += character;
            backslashes = 0;
        } else {
            quoted.append(backslashes, L'\\');
            quoted += character;
            backslashes = 0;
        }
    }
    quoted.append(backslashes * 2, L'\\');
    quoted += L'"';
    return quoted;
}

// Runs a tool to completion with stdout and stderr captured together; synchronous, no threads (determinism).
ProcessResult run_process(const std::vector<std::string>& arguments) {
    ProcessResult result;
    SECURITY_ATTRIBUTES inherit{};
    inherit.nLength = sizeof(inherit);
    inherit.bInheritHandle = TRUE;
    HANDLE read_end = nullptr;
    HANDLE write_end = nullptr;
    if (::CreatePipe(&read_end, &write_end, &inherit, 0) == 0) {
        result.output = "pipe failed: " + error_text(::GetLastError());
        return result;
    }
    ::SetHandleInformation(read_end, HANDLE_FLAG_INHERIT, 0);

    std::wstring line;
    for (const std::string& argument : arguments) {
        if (!line.empty()) {
            line += L' ';
        }
        line += quote_argument(widen(argument));
    }

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = ::GetStdHandle(STD_INPUT_HANDLE);
    startup.hStdOutput = write_end;
    startup.hStdError = write_end;
    PROCESS_INFORMATION process{};
    const BOOL spawned = ::CreateProcessW(nullptr, line.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                                          nullptr, &startup, &process);
    const DWORD spawn_error = ::GetLastError();
    ::CloseHandle(write_end);
    if (spawned == 0) {
        ::CloseHandle(read_end);
        result.output = "cannot start " + arguments.front() + ": " + error_text(spawn_error);
        return result;
    }

    std::array<char, 4096> buffer{};
    while (true) {
        DWORD count = 0;
        const BOOL read = ::ReadFile(read_end, buffer.data(), static_cast<DWORD>(buffer.size()), &count, nullptr);
        if (read == 0 || count == 0) {
            break;
        }
        result.output.append(buffer.data(), static_cast<std::size_t>(count));
    }
    ::CloseHandle(read_end);

    ::WaitForSingleObject(process.hProcess, INFINITE);
    DWORD exit_code = 1;
    if (::GetExitCodeProcess(process.hProcess, &exit_code) == 0) {
        result.output += "GetExitCodeProcess failed: " + error_text(::GetLastError());
        exit_code = 1;
    }
    ::CloseHandle(process.hThread);
    ::CloseHandle(process.hProcess);
    result.succeeded = exit_code == 0;
    return result;
}
#else
// Runs a tool to completion with stdout and stderr captured together; synchronous, no threads (determinism).
ProcessResult run_process(std::vector<std::string> arguments) {
    ProcessResult result;
    std::array<int, 2> pipe_ends{-1, -1};
    if (::pipe(pipe_ends.data()) != 0) {
        result.output = std::string("pipe failed: ") + std::strerror(errno);
        return result;
    }
    posix_spawn_file_actions_t actions{};
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, pipe_ends[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, pipe_ends[1], STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, pipe_ends[0]);
    posix_spawn_file_actions_addclose(&actions, pipe_ends[1]);

    std::vector<char*> argv;
    argv.reserve(arguments.size() + 1);
    for (std::string& argument : arguments) {
        argv.push_back(argument.data());
    }
    argv.push_back(nullptr);

    pid_t pid = 0;
    const int spawned = posix_spawnp(&pid, argv.front(), &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(pipe_ends[1]);
    if (spawned != 0) {
        ::close(pipe_ends[0]);
        result.output = "cannot start " + arguments.front() + ": " + std::strerror(spawned);
        return result;
    }

    std::array<char, 4096> buffer{};
    while (true) {
        const ssize_t count = ::read(pipe_ends[0], buffer.data(), buffer.size());
        if (count > 0) {
            result.output.append(buffer.data(), static_cast<std::size_t>(count));
        } else if (count == 0 || errno != EINTR) {
            break;
        }
    }
    ::close(pipe_ends[0]);

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) {
            result.output += std::string("waitpid failed: ") + std::strerror(errno);
            return result;
        }
    }
    result.succeeded = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    return result;
}
#endif

void run_step(const std::vector<std::string>& arguments, std::string& log) {
    const std::string line = command_line(arguments);
    const ProcessResult result = run_process(arguments);
    log += "$ " + line + "\n" + result.output;
    if (!result.succeeded) {
        throw ControllerConfigError("native firmware build failed at: " + line + "\n" + result.output);
    }
}

fs::path find_library(const fs::path& build_dir, const std::string& target) {
#if defined(_WIN32)
    const std::string name = target + ".dll";
#else
    const std::string name = "lib" + target + ".so";
#endif
    std::vector<fs::path> found;
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(build_dir)) {
        if (entry.is_regular_file() && entry.path().filename() == name) {
            found.push_back(entry.path());
        }
    }
    if (found.size() != 1) {
        throw ControllerConfigError("native firmware build: expected one " + name + " under " + build_dir.string() +
                                    ", found " + std::to_string(found.size()));
    }
    return found.front();
}

}  // namespace

std::string native_library_name(const std::string& app_target) {
#if defined(_WIN32)
    return app_target + "_native.dll";
#else
    return "lib" + app_target + "_native.so";
#endif
}

HostBuildResult build_host_firmware(const HostBuildRequest& request) {
    if (request.tick_hz == 0) {
        throw ControllerConfigError("native firmware build: tick rate must be positive");
    }
    if (request.app_target.empty() || request.work_dir.empty()) {
        throw ControllerConfigError("native firmware build: application target and work directory are required");
    }
    const fs::path tree = fs::absolute(request.firmware_tree).lexically_normal();
    if (!fs::is_regular_file(tree / "CMakePresets.json")) {
        throw ControllerConfigError("native firmware build: " + tree.string() + " has no CMakePresets.json");
    }
    const fs::path build_dir = (fs::absolute(request.work_dir) / "native-firmware" / "host").lexically_normal();
    std::error_code error;
    fs::create_directories(build_dir, error);
    if (error) {
        throw ControllerConfigError("native firmware build: cannot create " + build_dir.string() + ": " +
                                    error.message());
    }

    // Each app's host build has a SHARED <app>_native target (firmware/CMakeLists.txt) exporting the entry points.
    const std::string native_target = request.app_target + "_native";
    HostBuildResult result;
    run_step({request.cmake_command, "--preset", "host", "-S", tree.string(), "-B", build_dir.string(),
              "-DMECHSIM_APP_TICK_HZ=" + std::to_string(request.tick_hz), "-DMECHSIM_FW_BUILD_TESTS=OFF"},
             result.log);
    run_step({request.cmake_command, "--build", build_dir.string(), "--target", native_target}, result.log);
    result.library = find_library(build_dir, native_target);
    return result;
}

}  // namespace mechsim::controller
