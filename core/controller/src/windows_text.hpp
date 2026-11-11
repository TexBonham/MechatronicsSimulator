#pragma once

// Windows-only text helpers shared by the host build and the native runtime: UTF-8 <-> UTF-16 and system error text.
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <cstddef>
#include <string>

namespace mechsim::controller::detail {

inline std::wstring widen(const std::string& text) {
    if (text.empty()) {
        return {};
    }
    const int size = ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    std::wstring wide(static_cast<std::size_t>(size), L'\0');
    ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), wide.data(), size);
    return wide;
}

inline std::string narrow(const std::wstring& wide) {
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

inline std::string error_text(DWORD code) {
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

}  // namespace mechsim::controller::detail
#endif
