#include <windows.h>

typedef int (*export_fn)(void* scene, const char* outpath);

// returns:
//   0  = success (fn returned success code 0)
//   1  = fn returned non-success (export failed normally)
//  -1  = crash occurred (access violation, etc)
__declspec(dllexport) int safe_export_call(export_fn fn, void* scene, const char* outpath) {
    int rc = 1;
    __try {
        rc = fn(scene, outpath);
    }
    __except (EXCEPTION_EXECUTE_HANDLER) {
        rc = -1;
    }
    return rc;
}