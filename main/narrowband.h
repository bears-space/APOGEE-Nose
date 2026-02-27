#ifdef __cplusplus
extern "C" {
#endif

void init_radio(void);
void send_radio(const char* data, int len);
int  recv_radio(char* buf, int len);
void destroy_radio(void);

#ifdef __cplusplus
}
#endif