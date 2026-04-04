// packet_manager.cpp
// Implementation notes (the bulk of logic lives in packet_manager.h as a
// header-only class for embedded portability; this file provides non-inline
// helpers and explicit template instantiations if needed in the future).

#include "../include/packet_manager.h"

// PacketManager is fully defined in packet_manager.h (header-only for
// embedded targets that may not have a separate compilation step).
// This .cpp is the place to add:
//   1. Explicit template instantiations
//   2. Out-of-line implementations of large methods (move from .h if binary
//      size becomes a concern)
//   3. Platform-specific memory allocation overrides

namespace rmavlink {

// Example: factory helper so callers don't need to include the full header.
PacketManager* create_packet_manager(ITransport* transport) {
    return new PacketManager(transport);
}

void destroy_packet_manager(PacketManager* mgr) {
    delete mgr;
}

} // namespace rmavlink
