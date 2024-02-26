// Ref: https://gist.github.com/maxlapshin/1253534
// Ref: https://blog.csdn.net/tugouxp/article/details/118516385
// Ref: https://unicornx.github.io/2016/03/12/20160312-v4l2-app/


#include "cap.h"
//#include "utils.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


#include <pthread.h>
#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/videodev2.h>


#define CLEAR(x) memset(&(x), 0, sizeof(x))




// static void errno_exit(const char *s) {
//     fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
//     exit(EXIT_FAILURE);
// }

static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}


#define MAX_ID 10000

static int new_image_id;
static void *new_image_p;
static int new_image_size;
static bool new_image_flag;


static pthread_mutex_t new_image_meta_lock;
static pthread_cond_t new_image_sig;

static void process_image(void *p, int size) {
    pthread_mutex_lock(&new_image_meta_lock);
    if (new_image_flag) printf("An image ignored, processing too slow\n");
    ++new_image_id;
    new_image_p = p;
    new_image_size = size;
    new_image_flag = true;
    pthread_cond_signal(&new_image_sig);
    pthread_mutex_unlock(&new_image_meta_lock);
}

static pthread_t tid;

static void thread(void) {
    char *dev_name = "/dev/video0";

    // open_device
    // 打开设备，并且检测设备的一些基本参数
    struct stat st;
    int ret = stat(dev_name, &st);
    assert(ret != -1); // cannot identify
    assert(S_ISCHR(st.st_mode)); // not a device file

    int fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
    assert(fd != -1); // cannot open

    // init_device
    struct v4l2_capability cap;
    ret = xioctl(fd, VIDIOC_QUERYCAP, &cap);
    assert(ret != -1); // errno == EINVAL ? no V4L2 device : errno_exit("VIDIOC_QUERYCAP");
    assert(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE); // no video capture device

    // using mmap as io method
    assert(cap.capabilities & V4L2_CAP_STREAMING); // not support streaming i/o

    /* Select video input, video standard and tune here. */
    struct v4l2_cropcap cropcap;
    CLEAR(cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    struct v4l2_crop crop;
    ret = xioctl(fd, VIDIOC_CROPCAP, &cropcap);
    if (ret == 0) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        ret = xioctl(fd, VIDIOC_S_CROP, &crop);
        if (ret == -1) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }

    // 读取设备的控制参数，并且根据需要，调整这些参数并写回
    struct v4l2_format fmt;
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* Preserve original settings as set by v4l2-ctl for example */
    ret = xioctl(fd, VIDIOC_G_FMT, &fmt);
    assert(ret != -1); // errno_exit("VIDIOC_G_FMT");

    // set custom image meta
    fmt.fmt.pix.width = CAP_WIDTH;
    fmt.fmt.pix.height = CAP_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    ret = xioctl(fd, VIDIOC_S_FMT, &fmt);
    assert(ret != -1); // errno_exit("VIDIOC_S_FMT");
    ret = xioctl(fd, VIDIOC_G_FMT, &fmt);
    assert(ret != -1); // errno_exit("VIDIOC_G_FMT");
    assert(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG);

    /* Buggy driver paranoia. */
    unsigned int min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    // ref: https://gist.github.com/TIS-Edgar/e2bd764e5594897d1835
    struct v4l2_streamparm parm;
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = CAP_FPS;
    ret = xioctl(fd, VIDIOC_S_PARM, &parm);
    assert(ret != -1);

    // init_mmap
    // 初始化 mmap
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ret = xioctl(fd, VIDIOC_REQBUFS, &req);
    assert(ret != -1); // EINVAL == errno ? not support memory mapping : errno_exit("VIDIOC_REQBUFS");
    // assert(req.count >= 2); // Insufficient buffer memory
    assert(req.count >= 4); // 4 个必须都申请到，否则会对 grab 的 3 周期产生影响（来不及取回图像）

    struct buffer_s {
        void *start;
        size_t length;
    };
    struct buffer_s *buffers = calloc(req.count, sizeof(*buffers));
    assert(buffers); // Out of memory

    unsigned int n_buffers;
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        ret = xioctl(fd, VIDIOC_QUERYBUF, &buf);
        assert(ret != -1); // errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(
            NULL /* start anywhere */,
            buf.length,
            PROT_READ | PROT_WRITE /* required */,
            MAP_SHARED /* recommended */,
            fd,
            buf.m.offset
        );
        assert(buffers[n_buffers].start != MAP_FAILED); // errno_exit("mmap")
    }

    // start_capturing
    // 往 V4L2 的缓冲区队列里加缓冲区
    for (unsigned int i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = xioctl(fd, VIDIOC_QBUF, &buf);
        assert(ret != -1); // errno_exit("VIDIOC_QBUF");
    }

    // 开始流采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = xioctl(fd, VIDIOC_STREAMON, &type);
    assert(ret != -1); // errno_exit("VIDIOC_STREAMON");

    // mainloop
    // unsigned int count = 1100;
    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv;
        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        // 堵塞，直到有新的缓冲区填满
        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        assert(r != 0); // select timeout
        if (r == -1) {
            if (EINTR == errno)
                continue;
            else
                assert(false); // errno_exit("select");
        }

        // read_frame
        // 取回填满的缓冲区
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        ret = xioctl(fd, VIDIOC_DQBUF, &buf);
        if (ret == -1) {
            if (errno == EAGAIN) { // errno is thread-safe, ref: https://stackoverflow.com/questions/1694164/is-errno-thread-safe
                // nanosleep((const struct timespec[]){{0, 6000000L}}, NULL); // sleep 6ms
                // 这个睡眠时间跟摄像头 fps (120) 相关，太短会导致开销变大，太长会导致图片在缓冲区中堆积
                // 用 select 时可以省去这个函数
                continue; /* EAGAIN - continue select loop. */
            } else if (errno == EIO) {
                /* Could ignore EIO, see spec. */
                /* fall through */
            } else {
                assert(false); // errno_exit("VIDIOC_DQBUF"); // 这里出问题一般是摄像头掉了
            }
        }
        assert(buf.index < n_buffers);

        // // 时间测量
        // static struct timespec last_time1 = {0, 0};
        // struct timespec time1 = {0, 0};
        // clock_gettime(CLOCK_MONOTONIC, &time1);
        // printf("frame: %04ldus\n", (time1.tv_sec - last_time1.tv_sec) * 1000000 + (time1.tv_nsec - last_time1.tv_nsec) / 1000);
        // last_time1 = time1;

        // printf("index: %d\n", buf.index);
        process_image(buffers[buf.index].start, buf.bytesused);

        // 把用完的缓冲区再放回队列中
        // 实际上，真正的使用是在 grab 过程，这里并没有真正用完
        // 故从放回缓冲区开始，到 grab 使用完成之间的时间要小于 3个图像周期，
        // 否则这个缓冲区会被新的数据覆盖掉！
        ret = xioctl(fd, VIDIOC_QBUF, &buf);
        assert(ret != -1); // errno_exit("VIDIOC_QBUF");

        // if (count-- <= 0) break;
    }

    // stop_capturing
    // 停止流采集
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = xioctl(fd, VIDIOC_STREAMOFF, &type);
    assert(ret != -1); // errno_exit("VIDIOC_STREAMOFF");

    // uninit_device
    // 反初始化 mmap
    unsigned int i;
    for (i = 0; i < n_buffers; ++i) {
        ret = munmap(buffers[i].start, buffers[i].length);
        assert(ret != -1); // errno_exit("munmap");
    }

    free(buffers);

    // close_device
    ret = close(fd);
    assert(ret != -1); // errno_exit("close");

    fd = -1;
}


static bool is_inited, is_replay, is_record, have_meta, meta_not_processed;
static FILE *fp;
static FILE *fp_index;

static int replay_ids[MAX_ID];
static int replay_sizes[MAX_ID];
static int replay_meta_sizes[MAX_ID];
static int replay_offsets[MAX_ID + 1];
static int replay_ids_i;
static int replay_ids_n;

bool cap_init(bool is_replay_, bool is_record_, bool have_meta_) {
    assert(!is_inited);
    is_inited = true;

    is_replay = is_replay_;
    is_record = is_record_;
    have_meta = have_meta_;
    
    meta_not_processed = false;


        pthread_mutex_init(&new_image_meta_lock, NULL);

        // 这个线程会不断取回图像（不管图像用或者没有用），
        // 以尽量清空缓冲区，使得获取的图像最新
        // TODO 提升该线程优先级
        // ref: https://sites.google.com/site/myembededlife/Home/applications--development/linux-multi-thread-programming
        int ret = pthread_create(&tid, NULL, (void *(*)(void *))thread, NULL);

    return false;
}

bool cap_deinit(void) {
    assert(is_inited);
    is_inited = false;

    if (!is_replay) {
#ifdef WITH_GNU_COMPILER
        if (is_record) {
            fclose(fp);
            fclose(fp_index);
        }

        int ret = pthread_join(tid, NULL);
        assert(ret == 0);
#else
        assert(false);
#endif
    } else {
        fclose(fp);
    }
    return false;
}

bool cap_grab(void *raw_image, int *raw_image_size) {
    assert(is_inited);



        // // 时间测量
        // struct timespec time0 = {0, 0};
        // clock_gettime(CLOCK_MONOTONIC, &time0);

        pthread_mutex_lock(&new_image_meta_lock);
        while (new_image_flag != true)
            // ref: https://www.cnblogs.com/zhangxuan/p/6526854.html
            pthread_cond_wait(&new_image_sig, &new_image_meta_lock);
        void *p = new_image_p;
        int size = new_image_size;
        int image_id = new_image_id;
        new_image_flag = false;
        pthread_mutex_unlock(&new_image_meta_lock);

        // 接下来的过程要确保在 3 个图像周期内复制完图像，否则 v4l2 驱动会覆盖现在正在使用的图像
        // 3周期内取回就是线程安全，否则就不是
        // 时间测量
        struct timespec time1 = {0, 0};
        clock_gettime(CLOCK_MONOTONIC, &time1);

        memcpy(raw_image, p, size);
        *raw_image_size = size;

        // 时间测量
        // ref: https://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
        // ref: https://www.cnblogs.com/buptmuye/p/3711022.html
        struct timespec time2 = {0, 0};
        clock_gettime(CLOCK_MONOTONIC, &time2);
        assert((time2.tv_sec - time1.tv_sec) * 1000000 + (time2.tv_nsec - time1.tv_nsec) / 1000 < (3 - 0.5/*留出余量*/) * 1000000 / CAP_FPS); // 3 个图像周期内


    return false;
}




