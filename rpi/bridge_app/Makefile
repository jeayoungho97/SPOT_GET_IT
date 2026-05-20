CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -pthread -D_GNU_SOURCE
LIBS    = -pthread -lrt -lm

SRCS    = bridge_main.c jetson_rx.c jetson_tx.c reassembly_shm.c protocol_timer.c pc_link.c cmd_dispatch.c bridge_api.c shm_cmd_queue.c rx_packet_pool.c
TARGET  = bridge_daemon

$(TARGET): $(SRCS) shm_def.h proto.h frag_index_queue.h rx_packet_pool.h bridge_ctx.h cmd_dispatch.h bridge_api.h shm_cmd_queue.h utils.h
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LIBS)
	@echo "빌드 완료: $(TARGET)"

asan:
	$(CC) $(CFLAGS) -g -fsanitize=address \
	    $(SRCS) -o $(TARGET)_asan $(LIBS)
	@echo "빌드 완료: $(TARGET)_asan"

tsan:
	$(CC) $(CFLAGS) -g -fsanitize=thread \
	    $(SRCS) -o $(TARGET)_tsan $(LIBS)
	@echo "빌드 완료: $(TARGET)_tsan"

clean:
	rm -f $(TARGET) $(TARGET)_asan $(TARGET)_tsan

.PHONY: clean asan tsan
