/*
 * Copyright (C) 2010 The Android Open Source Project
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

#ifndef A_BIT_READER_H_

#define A_BIT_READER_H_

#include <sys/types.h>
#include <stdint.h>

namespace codec_utils {

struct ABitReader {
    ABitReader(const uint8_t *data, size_t size);

    uint32_t getBits(size_t n);
    void skipBits(size_t n);

    void putBits(uint32_t x, size_t n);

    size_t numBitsLeftInTotal() const;

    size_t numBitsLeftInPart() const {
        return mNumBitsLeft;
    };

    const uint8_t *data() const;

    int32_t getOffset() const;

private:
    const uint8_t *const mpInitial;
    const uint8_t *mpData;
    size_t mSize;

    uint32_t mReservoir;  // left-aligned bits
    size_t mNumBitsLeft;

    void fillReservoir();

    //DISALLOW_EVIL_CONSTRUCTORS(ABitReader);
};

}  // namespace codec_utils

#endif  // A_BIT_READER_H_
