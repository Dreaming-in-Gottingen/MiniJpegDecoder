#include <types_def.h>

namespace codec_utils {

uint64_t ntoh64(uint64_t x) {
    return ((uint64_t)ntohl(x & 0xffffffff) << 32) | ntohl(x >> 32) ;
}

uint64_t htoh64(uint64_t x) {
    return ((uint64_t)ntohl(x & 0xffffffff) << 32) | ntohl(x >> 32) ;
}

}  // namespace codec_utils
