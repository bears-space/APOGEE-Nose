#ifdef __cplusplus
extern "C" {
#endif
 
#include <stddef.h>
#include <sys/types.h>

void init_narrowband(void);
void enqueue_command(const uint8_t* data, size_t len);
// Dequeue into caller-provided buffer; returns number of bytes written,
// 0 if no data, -1 on error or if buffer is too small
ssize_t dequeue_command(uint8_t* buf, size_t max_len);
void handle_receive(void);
void narrowband_thread(void);
void destroy_narrowband(void);

#ifdef __cplusplus
}
#endif