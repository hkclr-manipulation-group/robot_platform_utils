#ifndef RET_CODE_H
#define RET_CODE_H

namespace robot::platform {
    enum class RetCode {
        kSuccess = 0,
        kFailed = 1,
        kInvalidInput = 2,
        kInvalidIndex = 3
    };
} // namespace robot::platform
#endif