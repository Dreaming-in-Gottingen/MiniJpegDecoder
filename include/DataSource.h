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

#ifndef DATA_SOURCE_H_

#define DATA_SOURCE_H_

#include <sys/types.h>

#include "ABase.h"
#include "AString.h"
#include "types_def.h"

namespace codec_utils {

class DataSource {
public:
    enum Flags {
        kWantsPrefetching      = 1,
        kStreamedFromLocalHost = 2,
        kIsCachingDataSource   = 4,
        kIsHTTPBasedSource     = 8,
    };

    virtual void getFd(int *fd, int64_t *offset) {
        *fd = -1;
        *offset = 0;
    }

    DataSource() {}

    virtual status_t initCheck() const = 0;

    virtual ssize_t readAt(off64_t offset, void *data, size_t size) = 0;

    // Convenience methods:
    bool getUInt16(off64_t offset, uint16_t *x);
    bool getUInt24(off64_t offset, uint32_t *x); // 3 byte int, returned as a 32-bit int
    bool getUInt32(off64_t offset, uint32_t *x);
    bool getUInt64(off64_t offset, uint64_t *x);

    // May return ERROR_UNSUPPORTED.
    virtual status_t getSize(off64_t *size);

    virtual uint32_t flags() {
        return 0;
    }

    ////////////////////////////////////////////////////////////////////////////
    //do not FORGET to delete raw-pointer
    static DataSource* CreateFromURI(const char *uri);

    virtual AString getUri() {
        return AString();
    }

    virtual AString getMIMEType() const {
        return AString("application/octet-stream");
    }

protected:
    virtual ~DataSource() {}

private:
    //DISALLOW_EVIL_CONSTRUCTORS(DataSource);
    DataSource(const DataSource &);
    DataSource &operator=(const DataSource &);
};

}  // namespace codec_utils

#endif  // DATA_SOURCE_H_
