/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "ADebug.h"
#include "FileSource.h"

namespace codec_utils {

FileSource::FileSource(const char *filename)
    : mFd(-1),
      mOffset(0),
      mLength(-1) {

#if defined(_WIN32) || defined(_WIN64)
    /* avoid read '0x0A' from '0x0D' '0x0A' on Windows platform */
    mFd = open(filename, O_RDONLY|O_BINARY);
#else
    mFd = open(filename, O_RDONLY);
#endif

    if (mFd >= 0) {
        mLength = lseek64(mFd, 0, SEEK_END);
    } else {
        fprintf(stderr, "Failed to open file '%s'. (%s)", filename, strerror(errno));
    }
}

FileSource::FileSource(int fd, int64_t offset, int64_t length)
    : mFd(fd),
      mOffset(offset),
      mLength(length) {

    CHECK(offset >= 0);
    CHECK(length >= 0);
}

FileSource::~FileSource() {
    if (mFd >= 0) {
        close(mFd);
        mFd = -1;
    }
}

status_t FileSource::initCheck() const {
    return mFd >= 0 ? OK : NO_INIT;
}

void FileSource::getFd(int *fd,int64_t *offset) {
    *fd = mFd;
    *offset = mOffset;
}

ssize_t FileSource::readAt(off64_t offset, void *data, size_t size) {
    if (mFd < 0) {
        return NO_INIT;
    }

    //Mutex::Autolock autoLock(mLock);

    if (mLength >= 0) {
        if (offset >= mLength) {
            return 0;  // read beyond EOF.
        }
        int64_t numAvailable = mLength - offset;
        if ((int64_t)size > numAvailable) {
            size = numAvailable;
        }
    }

    off64_t result = lseek64(mFd, offset + mOffset, SEEK_SET);
    if (result == -1) {
        fprintf(stderr, "seek to %lld failed", offset + mOffset);
        return UNKNOWN_ERROR;
    }
    //int64_t start_read = systemTime();
    int ret=::read(mFd, data, size);
    //int64_t time_consumed = systemTime() - start_read;
    //if (time_consumed >= 1000000000L) {
    //    ALOGE("io read used %lld ms",time_consumed/1000000L);
    //}

    return ret;
}

#define GET_SIZE_MAGIC 0xDEADBEEF
status_t FileSource::getSize(off64_t *size) {
    //Mutex::Autolock autoLock(mLock);
    if (mFd < 0) {
        return NO_INIT;
    }

    *size = mLength;

    return OK;
}

}  // namespace codec_utils
