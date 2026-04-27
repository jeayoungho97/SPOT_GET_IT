CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -pthread -D_GNU_SOURCE
LIBS    = -pthread -lrt -lm

SRCS    = bridge_main.c jetson_rx.c jetson_tx.c reassembly_shm.c protocol_timer.c pc_link.c cmd_dispatch.c
TARGET  = bridge_daemon

$(TARGET): $(SRCS) shm_def.h proto.h frag_queue.h bridge_ctx.h cmd_dispatch.h utils.h
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LIBS)
	@echo "빌드 완료: $(TARGET)"

# AddressSanitizer + ThreadSanitizer (디버깅용)
debug:
	$(CC) $(CFLAGS) -g -fsanitize=thread,address \
	    $(SRCS) -o $(TARGET)_debug $(LIBS)
	@echo "빌드 완료: $(TARGET)_debug"

clean:
	rm -f $(TARGET) $(TARGET)_debug

.PHONY: clean debug
