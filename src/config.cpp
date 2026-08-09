#include "frontier/config.h"

#if defined(_WIN32)
  #include <malloc.h>
#endif

namespace frontier {

void* defaultAlloc(size_t bytes, size_t alignment, void*)
{
    if (bytes == 0) return nullptr;
#if defined(_WIN32)
    return _aligned_malloc(bytes, alignment);
#else
    // aligned_alloc requires the size to be a multiple of the alignment.
    const size_t rounded = (bytes + alignment - 1) & ~(alignment - 1);
    return std::aligned_alloc(alignment, rounded);
#endif
}

void defaultFree(void* ptr, void*)
{
    if (!ptr) return;
#if defined(_WIN32)
    _aligned_free(ptr);
#else
    std::free(ptr);
#endif
}

void defaultParallelFor(uint32_t count, void (*fn)(uint32_t i, void* payload),
                        void* payload, void*)
{
    for (uint32_t i = 0; i < count; ++i) fn(i, payload);
}

const FrontierContext& defaultContext()
{
    static const FrontierContext ctx{};
    return ctx;
}

} // namespace frontier
