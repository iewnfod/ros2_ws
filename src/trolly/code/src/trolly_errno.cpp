#include "trolly/trolly_errno.h"

std::string trolly::err_to_string(trolly::err_t err_code)
{
    switch (err_code) {
        TROLLY_FOREACH_ERR(TROLLY_GENERATE_CASE)
        default:
            return "TROLLY_ERR_UNKNOWN";
    }
}
