//
// Created by lei on 10/26/2023.
//

#ifndef PARTICLESYSTEM_FILE_H
#define PARTICLESYSTEM_FILE_H

#pragma once

#define __STDC_WANT_LIB_EXT1__ 1

#include <string>
#include <cstring>
#include <sstream>
#include <iostream>

#if defined(WIN32) || defined(_WIN32)

#include <io.h>

#elif defined(__linux__)

#include <sys/io.h>

#define strtok_s strtok_r

#include<unistd.h>

#define _access access

#endif

#if defined(WIN32) || defined(_WIN32)
constexpr char DELIMITER = '\\';
#else
constexpr char DELIMITER = '/';
#endif

namespace utils::file {
    using std::string;

    template<typename T = string>
    string concatFilePath(T firstArg) {
        std::stringstream ss;
        ss << firstArg;
        return ss.str();
    }

    template<typename T = string, typename...Types>
    string concatFilePath(T firstArg, Types ... args) {
        std::stringstream ss;
        ss << firstArg << DELIMITER << concatFilePath(args...);
        return ss.str();
    }

    inline
    char const *getFileNameWithExt(char *filePath) {
        char *fileName = filePath;
        const size_t len = strlen(filePath);
        for (int i = 0; i < len; ++i) {
            if (filePath[i] == DELIMITER)
                fileName = &filePath[i + 1];
        }
        return fileName;
    }

    inline
    char const *getFileNameWithExt(const char *filePath) {
        char *filePath_c = new char[strlen(filePath) + 1];
        strcpy(filePath_c, filePath);
        char *fileName = nullptr;

#ifdef __STDC_LIB_EXT1__
        char* token = strtok(filePath_c, &delimiter); // 'filePath_c' is changed after calles strtok
            while (token != nullptr)
            {
                if (fileName != nullptr) { delete[] fileName; fileName = nullptr; }
                fileName = new char[strlen(token) + 1];
                strcpy(fileName, token);
                token = strtok(NULL, &delimiter);
            }
#else // Thread safe
        char *ptr = nullptr;
        char *token = strtok_s(filePath_c, &DELIMITER,
                               &ptr);
        while (token != nullptr) {
            if (fileName != nullptr) {
                delete[] fileName;
                fileName = nullptr;
            }
            fileName = new char[strlen(token) + 1];
            strcpy(fileName, token);
            token = strtok_s(nullptr, &DELIMITER, &ptr);
        }
#endif
        delete[] filePath_c;
        filePath_c = nullptr;
        return fileName;
    }

    inline
    string getFileNameWithExt(const string &filePath) {
        const string fileName = getFileNameWithExt(filePath.c_str());
        return fileName;
    }

    // such as ".obj"
    inline
    char const *getFileExtension(char *filePath) {
        char *extension = filePath;
        for (int i = strlen(filePath); i >= 0; --i) {
            if (filePath[i] == '.') {
                extension = &filePath[i];
                return extension;
            }
        }
        extension[0] = 0;
        return extension;
    }

    // such as '.obj'
    inline
    char const *getFileExtension(const char *filePath) {
        char *temp = new char[strlen(filePath) + 1];
        strcpy(temp, filePath);
        const char *extension = getFileExtension(temp);
        delete[] temp;
        temp = nullptr;
        return extension;
    }

    inline
    string getFileExtension(const string &filePath) {
        const size_t idx = filePath.rfind('.');
        if (idx + 1 != string::npos)
            return filePath.substr(idx);
        return "";
    }

    inline
    char const *getFileName(char *filePath) {
        char const *fileNameWithExt = getFileNameWithExt(filePath);
        size_t len = strlen(fileNameWithExt);
        char *fileName = new char[len + 1];
        strncpy(fileName, fileNameWithExt, len);

        for (int i = len - 1; i >= 0; --i) {
            if (fileName[i] == '.') {
                fileName[i] = 0;
                return fileName;
            }
        }
        return fileName;
    }

    inline
    char const *getFileName(const char *filePath) {
        char *fileName = const_cast<char *>(getFileNameWithExt(filePath));
        size_t len = strlen(fileName);
        for (int i = len - 1; i >= 0; --i) {
            if (fileName[i] == '.') {
                fileName[i] = 0;
                return fileName;
            }
        }
        return fileName;
    }

    inline
    string getFileName(const string &filePath) {
        string fileName = getFileName(filePath.c_str());
        return fileName;
    }

    inline
    char const *getDirName(const char *filePath) {
        const size_t len = strlen(filePath);
        char *dirName = new char[len + 1];
        strcpy(dirName, filePath);
        for (int i = len - 1; i >= 0; --i) {
            if (filePath[i] == DELIMITER) {
                dirName[i] = 0;
                return dirName;
            }
        }
        dirName[0] = 0;
        return dirName;
    }

    inline
    char const *getDirName(char *filePath) {
        const size_t len = strlen(filePath);
        char *dirName = new char[len + 1];
        strcpy(dirName, filePath);
        for (int i = len - 1; i >= 0; --i) {
            if (filePath[i] == DELIMITER) {
                dirName[i] = 0;
                return dirName;
            }
        }
        dirName[0] = 0;
        return dirName;
    }

    inline
    string getDirName(const string &filePath) {
        string dirName = getDirName(filePath.c_str());
        return dirName;
    }

    inline
    void checkDir(const string &filename) {
        size_t dir_idx = filename.find_last_of(DELIMITER);
        if (dir_idx != string::npos) {
            string dir = filename.substr(0, dir_idx);
            if (_access(dir.c_str(), 0) == -1) {
                string command = "mkdir " + dir;
                int status = system(command.c_str());
                if (status != 0) std::cerr << "Failed to create directory. Exit status: " << status << std::endl;
            }
        }
    }

} // namespace utils::file

#endif //PARTICLESYSTEM_FILE_H
