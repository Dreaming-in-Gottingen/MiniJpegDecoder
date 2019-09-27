#ifndef TYPES_DEF_H_

#define TYPES_DEF_H_

#include <stdint.h>
#include <sys/types.h>
#include <errno.h>

namespace codec_utils {

#define HAVE_LITTLE_ENDIAN

# ifdef HAVE_LITTLE_ENDIAN
#  define ntohl(x)    ( ((x) << 24) | (((x) >> 24) & 255) | (((x) << 8) & 0xff0000) | (((x) >> 8) & 0xff00) )
#  define htonl(x)    ntohl(x)
#  define ntohs(x)    ( (((x) << 8) & 0xff00) | (((x) >> 8) & 255) )
#  define htons(x)    ntohs(x)
# else
#  define ntohl(x)    (x)
#  define htonl(x)    (x)
#  define ntohs(x)    (x)
#  define htons(x)    (x)
# endif

typedef int status_t;


uint64_t ntoh64(uint64_t x);

uint64_t htoh64(uint64_t x);

enum {
    OK                = 0,    // Everything's swell.
    NO_ERROR          = 0,    // No errors.

    NO_INIT             = -ENODEV,
    UNKNOWN_ERROR       = 0x80000000,

    MEDIA_ERROR_BASE        = -1000,

    ERROR_ALREADY_CONNECTED = MEDIA_ERROR_BASE,
    ERROR_NOT_CONNECTED     = MEDIA_ERROR_BASE - 1,
    ERROR_UNKNOWN_HOST      = MEDIA_ERROR_BASE - 2,
    ERROR_CANNOT_CONNECT    = MEDIA_ERROR_BASE - 3,
    ERROR_IO                = MEDIA_ERROR_BASE - 4,
    ERROR_CONNECTION_LOST   = MEDIA_ERROR_BASE - 5,
    ERROR_MALFORMED         = MEDIA_ERROR_BASE - 7,
    ERROR_OUT_OF_RANGE      = MEDIA_ERROR_BASE - 8,
    ERROR_BUFFER_TOO_SMALL  = MEDIA_ERROR_BASE - 9,
    ERROR_UNSUPPORTED       = MEDIA_ERROR_BASE - 10,
    ERROR_END_OF_STREAM     = MEDIA_ERROR_BASE - 11,
};

}  // namespace codec_utils

#endif //TYPES_DEF_H_
