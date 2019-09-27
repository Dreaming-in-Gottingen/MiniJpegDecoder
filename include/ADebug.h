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

#ifndef A_DEBUG_H_

#define A_DEBUG_H_

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#include "ABase.h"
#include "AString.h"

namespace codec_utils {

#define LITERAL_TO_STRING_INTERNAL(x)    #x
#define LITERAL_TO_STRING(x) LITERAL_TO_STRING_INTERNAL(x)

//#define CHECK(condition)                                \
//    LOG_ALWAYS_FATAL_IF(                                \
//            !(condition),                               \
//            "%s",                                       \
//            __FILE__ ":" LITERAL_TO_STRING(__LINE__)    \
//            " CHECK(" #condition ") failed.")

#define CHECK(condition)                                \
    do {                                                \
        if (!(condition)) {                             \
            puts("%s:"LITERAL_TO_STRING(__LINE__)       \
                    " CHECK(" #condition ") failed.");  \
            assert(0);                                  \
        }                                               \
    } while(0)

#define MAKE_COMPARATOR(suffix,op)                          \
    template<class A, class B>                              \
    AString Compare_##suffix(const A &a, const B &b) {      \
        AString res;                                        \
        if (!(a op b)) {                                    \
            res.append(a);                                  \
            res.append(" vs. ");                            \
            res.append(b);                                  \
        }                                                   \
        return res;                                         \
    }


MAKE_COMPARATOR(EQ,==)
MAKE_COMPARATOR(NE,!=)
MAKE_COMPARATOR(LE,<=)
MAKE_COMPARATOR(GE,>=)
MAKE_COMPARATOR(LT,<)
MAKE_COMPARATOR(GT,>)

//#define CHECK_OP(x,y,suffix,op)                                         \
//    do {                                                                \
//        AString ___res = Compare_##suffix(x, y);                        \
//        if (!___res.empty()) {                                          \
//            AString ___full =                                           \
//                __FILE__ ":" LITERAL_TO_STRING(__LINE__)                \
//                    " CHECK_" #suffix "( " #x "," #y ") failed: ";      \
//            ___full.append(___res);                                     \
//                                                                        \
//            LOG_ALWAYS_FATAL("%s", ___full.c_str());                    \
//        }                                                               \
//    } while (false)
//
#define CHECK_OP(x,y,suffix,op)                                         \
    do {                                                                \
        AString ___res = Compare_##suffix(x, y);                         \
        if (!___res.empty()) {                                          \
            AString ___full =                                            \
                __FILE__ ":" LITERAL_TO_STRING(__LINE__)                \
                " CHECK_" #suffix "(" #x "," #y ") failed: ";           \
            ___full.append(___res);                                     \
            puts(___full.c_str());                                      \
            assert(0);                                                  \
        }                                                               \
    } while(0)

#define CHECK_EQ(x,y)   CHECK_OP(x,y,EQ,==)
#define CHECK_NE(x,y)   CHECK_OP(x,y,NE,!=)
#define CHECK_LE(x,y)   CHECK_OP(x,y,LE,<=)
#define CHECK_LT(x,y)   CHECK_OP(x,y,LT,<)
#define CHECK_GE(x,y)   CHECK_OP(x,y,GE,>=)
#define CHECK_GT(x,y)   CHECK_OP(x,y,GT,>)

//#define TRESPASS() \
//        LOG_ALWAYS_FATAL(                                       \
//            __FILE__ ":" LITERAL_TO_STRING(__LINE__)            \
//                " Should not be here.");

#define TRESPASS()                                              \
    do {                                                        \
        puts(__FILE__ ":" LITERAL_TO_STRING(__LINE__)           \
                " Should not be here.");                        \
        assert(0);                                              \
    } while(0)

}  // namespace codec_utils

#endif  // A_DEBUG_H_

