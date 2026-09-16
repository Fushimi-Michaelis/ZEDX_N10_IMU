/* Read the 20-byte Stereolabs SPSC ABI. Timestamps are MONOTONIC estimates,
 * not hardware exposure times or calibrated UTC. Run as root on this N10. */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

struct sample { uint64_t stamp; int16_t gx,gy,gz,ax,ay,az; } __attribute__((packed));
struct ring { uint32_t head,tail; struct sample samples[1024]; };
struct row { struct sample data; uint64_t mono,raw,wall; };
_Static_assert(sizeof(struct sample)==20,"SPSC ABI mismatch");
static const char *base="/sys/class/bmi_spsc/spsc_bmi0";
static volatile sig_atomic_t interrupted;
static void stop(int sig) { (void)sig; interrupted=1; }
static uint64_t ns(clockid_t clock) {
    struct timespec t; if(clock_gettime(clock,&t)) { perror("clock_gettime"); exit(1); }
    return (uint64_t)t.tv_sec*1000000000ULL+t.tv_nsec;
}
static long get(const char *name) {
    char p[256]; snprintf(p,sizeof(p),"%s/%s",base,name);
    FILE *f=fopen(p,"r"); long v=-1;
    if (!f) return -1;
    if(fscanf(f,"%li",&v)!=1) v=-1;
    fclose(f); return v;
}
static int set(const char *name,int value) {
    char p[256],data[32]; snprintf(p,sizeof(p),"%s/%s",base,name);
    int fd=open(p,O_WRONLY); if(fd<0) { perror(p); return -1; }
    int len=snprintf(data,sizeof(data),"%d\n",value);
    ssize_t done=write(fd,data,(size_t)len); int saved=errno; close(fd);
    if(done!=len) { errno=saved; perror(p); return -1; }
    return 0;
}
int main(int argc,char **argv) {
    if(argc!=3) { fprintf(stderr,"Usage: %s NEW_DIRECTORY SECONDS(2..120)\n",argv[0]); return 1; }
    int secs=atoi(argv[2]); if(secs<2||secs>120) return 1;
    if(get("i2c_bus")!=7 || get("acc_addr")!=0x3c || get("gyro_addr")!=0x3b) {
        fprintf(stderr,"Wrong IMU identity\n"); return 1;
    }
    int fd=open("/dev/spsc_bmi0",O_RDWR); if(fd<0) { perror("IMU"); return 1; }
    long bytes=get("ring_buffer_size");
    if(bytes<(long)sizeof(struct ring)||bytes>1048576) { close(fd); return 1; }
    struct ring *r=mmap(NULL,(size_t)bytes,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    if(r==MAP_FAILED) { perror("mmap (already in use?)"); close(fd); return 1; }
    /* Acquire the sole consumer mapping before any control changes. */
    int result=1,active=0; size_t count=0, capacity=(size_t)secs*1000+1024;
    struct row *rows=calloc(capacity,sizeof(*rows));
    if(!rows || mkdir(argv[1],0755)) { perror("new output"); goto cleanup; }
    signal(SIGINT,stop); signal(SIGTERM,stop);
    if(set("timer_control",0)||set("accel_range",2)||set("gyro_range",1)||
       set("accel_frequency",200)||set("gyro_frequency",200)||set("timer_control",1)) goto cleanup;
    active=1;
    uint64_t begin=ns(CLOCK_MONOTONIC),end=begin+(uint64_t)secs*1000000000ULL;
    while(!interrupted && ns(CLOCK_MONOTONIC)<end) {
        uint32_t head=__atomic_load_n(&r->head,__ATOMIC_ACQUIRE);
        uint32_t tail=__atomic_load_n(&r->tail,__ATOMIC_RELAXED);
        if(head>=1024 || tail>=1024) { fprintf(stderr,"Corrupt ring\n"); goto cleanup; }
        if(head==tail) { struct timespec pause={.tv_nsec=1000000}; nanosleep(&pause,NULL); continue; }
        while(tail!=head) {
            if(count>=capacity) { fprintf(stderr,"Capture capacity exceeded\n"); goto cleanup; }
            struct row *row=&rows[count++];
            memcpy(&row->data,&r->samples[tail],sizeof(row->data));
            row->mono=ns(CLOCK_MONOTONIC); row->raw=ns(CLOCK_MONOTONIC_RAW); row->wall=ns(CLOCK_REALTIME);
            tail=(tail+1)&1023;
            __atomic_store_n(&r->tail,tail,__ATOMIC_RELEASE);
        }
    }
    if(set("timer_control",0)) goto cleanup;
    active=0;
    char path[1024]; snprintf(path,sizeof(path),"%s/imu.csv",argv[1]);
    FILE *f=fopen(path,"w"); if(!f) { perror(path); goto cleanup; }
    fprintf(f,"index,imu_monotonic_estimate_ns,receive_monotonic_ns,receive_raw_ns,receive_realtime_UNTRUSTED_ns,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw\n");
    for(size_t i=0;i<count;i++) {
        struct row *row=&rows[i]; struct sample *s=&row->data;
        fprintf(f,"%zu,%"PRIu64",%"PRIu64",%"PRIu64",%"PRIu64",%d,%d,%d,%d,%d,%d\n",
                i,s->stamp,row->mono,row->raw,row->wall,s->gx,s->gy,s->gz,s->ax,s->ay,s->az);
    }
    if(fclose(f)) goto cleanup;
    snprintf(path,sizeof(path),"%s/session.json",argv[1]);
    f=fopen(path,"w"); if(!f) goto cleanup;
    char boot[64]="unknown"; FILE *b=fopen("/proc/sys/kernel/random/boot_id","r");
    if(b) { if(!fgets(boot,sizeof(boot),b)) strcpy(boot,"unknown"); fclose(b); }
    boot[strcspn(boot,"\r\n")]=0;
    fprintf(f,"{\"boot_id\":\"%s\",\"samples\":%zu,\"duration_seconds\":%d,\"begin_monotonic_ns\":%"PRIu64",\"end_monotonic_ns\":%"PRIu64",\"utc_verified\":false,\"exposure_verified\":false,\"sample_time_is_estimated\":true,\"accel_range_g\":12,\"gyro_range_dps\":1000,\"accel_frequency\":%ld,\"gyro_frequency\":%ld,\"i2c_errors\":%ld,\"ring_drops\":%ld,\"fifo_overruns\":%ld,\"interrupted\":%s}\n",
            boot,count,secs,begin,ns(CLOCK_MONOTONIC),get("accel_frequency"),get("gyro_frequency"),get("i2c_errors"),get("ring_drops"),get("fifo_overruns"),interrupted?"true":"false");
    if(fclose(f)) goto cleanup;
    printf("IMU: %zu samples, %d seconds; timer stopped\n",count,secs);
    result=interrupted?2:(count?0:1);
cleanup:
    if(active) set("timer_control",0);
    munmap(r,(size_t)bytes); close(fd); free(rows); return result;
}
