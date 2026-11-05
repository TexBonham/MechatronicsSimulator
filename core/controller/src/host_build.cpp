#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

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

void run_step(const std::vector<std::string>& arguments, std::string& log) {
    const std::string line = command_line(arguments);
    const ProcessResult result = run_process(arguments);
    log += "$ " + line + "\n" + result.output;
    if (!result.succeeded) {
        throw ControllerConfigError("native firmware build failed at: " + line + "\n" + result.output);
    }
}

fs::path find_library(const fs::path& build_dir, const std::string& target) {
    const std::string name = "lib" + target + ".so";
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
