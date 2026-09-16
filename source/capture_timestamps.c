/* Stereo V4L2 diagnostic: bind each retained payload to its own DQBUF metadata.
 * No per-frame disk writes occur during streaming. Does not change system time.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/timex.h>
#include <time.h>
#include <unistd.h>

#define EYES 2
#define BUFS 8
struct mapping { void *ptr; size_t length; };
struct row {
    uint32_t sequence, flags, bytes, buffer;
    uint64_t stamp, receive_raw, receive_mono, receive_wall, counter, bracket;
    int64_t counter_offset, frame_raw;
    unsigned char *payload;
};
struct eye { int fd, streaming, count; struct mapping map[BUFS]; unsigned maps;
    struct row *rows; struct v4l2_format fmt; };

static uint64_t ns(clockid_t clock) {
    struct timespec t;
    if (clock_gettime(clock, &t)) { perror("clock_gettime"); exit(1); }
    return (uint64_t)t.tv_sec * 1000000000ULL + t.tv_nsec;
}
static int xioctl(int fd, unsigned long request, void *arg) {
    int rc; do { rc = ioctl(fd, request, arg); } while (rc < 0 && errno == EINTR);
    return rc;
}
static uint64_t frequency(void) { uint64_t v; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
static uint64_t counter_ns(uint64_t freq) {
    uint64_t v;
    __asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v) :: "memory");
    return (uint64_t)((__uint128_t)v * 1000000000ULL / freq);
}
static int setup(struct eye *eye, int number, int total) {
    char device[32]; snprintf(device, sizeof(device), "/dev/video%d", number);
    eye->fd = open(device, O_RDWR | O_NONBLOCK);
    if (eye->fd < 0) { perror(device); return -1; }
    struct v4l2_capability cap = {0};
    if (xioctl(eye->fd, VIDIOC_QUERYCAP, &cap) || !strstr((char *)cap.card, "zedx")) {
        fprintf(stderr, "%s is not a ZED capture node\n", device); return -1;
    }
    eye->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(eye->fd, VIDIOC_G_FMT, &eye->fmt)) return -1;
    if (eye->fmt.fmt.pix.width != 1920 || eye->fmt.fmt.pix.height != 1200 ||
        eye->fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_SGRBG10 ||
        eye->fmt.fmt.pix.bytesperline != 3840 || eye->fmt.fmt.pix.sizeimage != 4608000) {
        fprintf(stderr, "%s: configure 1920x1200 BA10 before capture\n", device); return -1;
    }
    struct v4l2_requestbuffers req = {.count=BUFS, .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP};
    if (xioctl(eye->fd, VIDIOC_REQBUFS, &req) || !req.count || req.count > BUFS) return -1;
    for (unsigned i=0; i<req.count; i++) {
        struct v4l2_buffer b = {.type=req.type, .memory=req.memory, .index=i};
        if (xioctl(eye->fd, VIDIOC_QUERYBUF, &b)) return -1;
        void *ptr = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, eye->fd, b.m.offset);
        if (ptr == MAP_FAILED) return -1;
        eye->map[i] = (struct mapping){ptr, b.length}; eye->maps++;
        if (xioctl(eye->fd, VIDIOC_QBUF, &b)) return -1;
    }
    eye->rows = calloc((size_t)total, sizeof(struct row));
    return eye->rows ? 0 : -1;
}
static void cleanup(struct eye *eye) {
    enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (eye->streaming) xioctl(eye->fd, VIDIOC_STREAMOFF, &type);
    for (unsigned i=0; i<eye->maps; i++) munmap(eye->map[i].ptr, eye->map[i].length);
    if (eye->fd >= 0) close(eye->fd);
}
int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr,"Usage: %s OUTPUT [frames=132] [warmup=12] [save=10]\n",argv[0]); return 1; }
    int total=argc>2?atoi(argv[2]):132, warmup=argc>3?atoi(argv[3]):12, keep=argc>4?atoi(argv[4]):10;
    if (total<2 || total>10000 || warmup<0 || keep<0 || keep>20 || warmup+keep>total) return 1;
    if (mkdir(argv[1],0755)) { perror("output directory (must be new)"); return 1; }
    struct eye eyes[EYES] = {{.fd=-1},{.fd=-1}};
    int result=0; uint64_t freq=frequency(), start=ns(CLOCK_MONOTONIC);
    char boot_id[64]="unknown";
    FILE *boot=fopen("/proc/sys/kernel/random/boot_id","r");
    if (boot) { if (!fgets(boot_id,sizeof(boot_id),boot)) strcpy(boot_id,"unknown"); fclose(boot); }
    boot_id[strcspn(boot_id,"\r\n")]=0;
    struct timex clock_state={0};
    int ntp_state=adjtimex(&clock_state);
    int synchronized=ntp_state>=0 && ntp_state!=TIME_ERROR && !(clock_state.status & STA_UNSYNC);
    if (!freq) return 1;
    for (int i=0;i<EYES;i++) if (setup(&eyes[i],4+i,total)) { perror("setup"); result=1; goto done; }
    for (int i=0;i<EYES;i++) {
        enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(eyes[i].fd,VIDIOC_STREAMON,&type)) { perror("STREAMON"); result=1; goto done; }
        eyes[i].streaming=1;
    }
    while (eyes[0].count < total || eyes[1].count < total) {
        if (ns(CLOCK_MONOTONIC)-start > 60000000000ULL) { fprintf(stderr,"Capture deadline exceeded\n"); result=2; break; }
        struct pollfd pollers[EYES];
        for (int i=0;i<EYES;i++) pollers[i]=(struct pollfd){.fd=eyes[i].count<total?eyes[i].fd:-1,.events=POLLIN};
        int ready=poll(pollers,EYES,3000);
        if (ready<0 && errno==EINTR) continue;
        if (ready<0) { perror("poll"); result=1; break; }
        for (int i=0;i<EYES;i++) {
            if (pollers[i].fd<0 || !(pollers[i].revents & (POLLIN|POLLERR))) continue;
            struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE,.memory=V4L2_MEMORY_MMAP};
            if (xioctl(eyes[i].fd,VIDIOC_DQBUF,&b)) {
                if (errno==EAGAIN) continue;
                perror("DQBUF"); result=1; goto done;
            }
            if (b.index>=eyes[i].maps || b.bytesused>eyes[i].map[b.index].length) { result=1; goto done; }
            struct row *r=&eyes[i].rows[eyes[i].count];
            uint64_t before=ns(CLOCK_MONOTONIC_RAW);
            r->counter=counter_ns(freq);
            uint64_t after=ns(CLOCK_MONOTONIC_RAW);
            r->receive_raw=(before+after)/2; r->bracket=after-before;
            r->receive_mono=ns(CLOCK_MONOTONIC); r->receive_wall=ns(CLOCK_REALTIME);
            r->stamp=(uint64_t)b.timestamp.tv_sec*1000000000ULL+(uint64_t)b.timestamp.tv_usec*1000ULL;
            r->counter_offset=(int64_t)r->counter-(int64_t)r->receive_raw;
            r->frame_raw=(int64_t)r->stamp-r->counter_offset;
            r->sequence=b.sequence; r->flags=b.flags; r->bytes=b.bytesused; r->buffer=b.index;
            if (eyes[i].count>=warmup && eyes[i].count<warmup+keep) {
                r->payload=malloc(b.bytesused);
                if (!r->payload) { result=1; goto done; }
                memcpy(r->payload,eyes[i].map[b.index].ptr,b.bytesused);
            }
            eyes[i].count++;
            if (xioctl(eyes[i].fd,VIDIOC_QBUF,&b)) { perror("QBUF"); result=1; goto done; }
        }
    }
done:
    for (int i=0;i<EYES;i++) cleanup(&eyes[i]);
    char session_path[1024]; snprintf(session_path,sizeof(session_path),"%s/session.json",argv[1]);
    FILE *session=fopen(session_path,"w");
    if (!session) return 1;
    fprintf(session,"{\"boot_id\":\"%s\",\"counter_frequency_hz\":%llu,\"timestamp_precision_ns\":1000,\"start_monotonic_ns\":%llu,\"end_monotonic_ns\":%llu,\"system_clock_synchronized\":%s,\"adjtimex_state\":%d,\"adjtimex_status\":%d,\"warmup_frames\":%d,\"requested_frames_per_eye\":%d,\"capture_return_code\":%d}\n",
        boot_id,(unsigned long long)freq,(unsigned long long)start,(unsigned long long)ns(CLOCK_MONOTONIC),
        synchronized?"true":"false",ntp_state,clock_state.status,warmup,total,result);
    fclose(session);
    for (int i=0;i<EYES;i++) {
        const char *side=i?"right":"left"; char path[1024];
        snprintf(path,sizeof(path),"%s/%s.csv",argv[1],side);
        FILE *csv=fopen(path,"w"); if (!csv) { perror(path); return 1; }
        fprintf(csv,"index,sequence,flags,bytesused,buffer_index,warmup,v4l2_timestamp_ns,counter_ns,receive_monotonic_raw_ns,receive_monotonic_ns,receive_realtime_ns,counter_to_raw_offset_ns,frame_monotonic_raw_estimate_ns,clock_sample_span_ns,raw_file\n");
        for (int j=0;j<eyes[i].count;j++) {
            struct row *r=&eyes[i].rows[j]; char name[80]="";
            if (r->payload) {
                snprintf(name,sizeof(name),"%s_seq%06u.raw",side,r->sequence);
                snprintf(path,sizeof(path),"%s/%s",argv[1],name);
                FILE *raw=fopen(path,"wb");
                if (!raw || fwrite(r->payload,1,r->bytes,raw)!=r->bytes) { perror("write RAW"); return 1; }
                if (fclose(raw)) return 1;
            }
            fprintf(csv,"%d,%u,%u,%u,%u,%d,%llu,%llu,%llu,%llu,%llu,%lld,%lld,%llu,%s\n",j,r->sequence,r->flags,r->bytes,r->buffer,j<warmup,
                (unsigned long long)r->stamp,(unsigned long long)r->counter,(unsigned long long)r->receive_raw,(unsigned long long)r->receive_mono,(unsigned long long)r->receive_wall,
                (long long)r->counter_offset,(long long)r->frame_raw,(unsigned long long)r->bracket,name);
            free(r->payload);
        }
        fclose(csv); free(eyes[i].rows);
        printf("%s: %d frames, counter frequency %llu Hz\n",side,eyes[i].count,(unsigned long long)freq);
    }
    return result;
}
