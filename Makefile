CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -I include
LDFLAGS  := -lpthread

# ── Build all ─────────────────────────────────
all: rmavlink packet_loss_sim

# ── Main demo ─────────────────────────────────
SRCS_MAIN := main.cpp \
             core/reliable_protocol.cpp \
             core/packet_manager.cpp \
             mavlink/mavlink_parser.cpp

rmavlink: $(SRCS_MAIN) include/reliable_protocol.h include/packet_manager.h
	$(CXX) $(CXXFLAGS) $(SRCS_MAIN) -o $@ $(LDFLAGS)
	@echo "[OK] Built ./rmavlink"

# ── Packet loss simulation ─────────────────────
SRCS_SIM := tests/packet_loss_sim.cpp

packet_loss_sim: $(SRCS_SIM) include/reliable_protocol.h include/packet_manager.h
	$(CXX) $(CXXFLAGS) $(SRCS_SIM) -o $@ $(LDFLAGS)
	@echo "[OK] Built ./packet_loss_sim"

# ── Run simulation ────────────────────────────
test: packet_loss_sim
	./packet_loss_sim

# ── Quick loopback demo ───────────────────────
demo: rmavlink
	@echo "Starting receiver in background..."
	./rmavlink recv &
	@sleep 1
	@echo "Starting sender..."
	./rmavlink send

# ── Clean ─────────────────────────────────────
clean:
	rm -f rmavlink packet_loss_sim *.o

.PHONY: all test demo clean
