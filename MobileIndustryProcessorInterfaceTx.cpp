// by felix

#include <stdlib.h>
#include <unistd.h>
#include <limits.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include "Log.h"
#include "MobileIndustryProcessorInterfaceTx.h"


namespace RK {

static int frame_num = 0;
const char *MipiRxDevName = "/dev/video0";
static enum v4l2_buf_type MipiRxBufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#define MIPI_RX_FMT_NUM_PLANES 1
#define MEMSET(x) memset(&(x), 0, sizeof(x))

#define GRALLOC_ALIGN( value, base ) (((value) + ((base) - 1)) & ~((base) - 1))

struct plane_prop {
    int crtc_id;
    int fb_id;

    int src_x;
    int src_y;
    int src_w;
    int src_h;

    int crtc_x;
    int crtc_y;
    int crtc_w;
    int crtc_h;

    int data_type;

    int zpos;
    int feature;
};

struct drm_planes {
    struct plane_prop plane_prop;
    drmModePlanePtr plane;

    int support_scale;
};

extern int isp_init(void);

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;
        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);
        return r;
}

/*
 * Computes the strides and size for an RGB buffer
 *
 * width               width of the buffer in pixels
 * height              height of the buffer in pixels
 * pixel_size          size of one pixel in bytes
 *
 * pixel_stride (out)  stride of the buffer in pixels
 * byte_stride  (out)  stride of the buffer in bytes
 * size         (out)  size of the buffer in bytes
 */
static void get_rgb_stride_and_size(int width, int height, int pixel_size,
                                    int *pixel_stride, int *byte_stride, int *size)
{
    int stride;

    stride = width * pixel_size;

    /* Align the lines to 64 bytes.
     * It's more efficient to write to 64-byte aligned addresses
     * because it's the burst size on the bus
     * at drm gem driver will align 64 byte, so we align 64 byte align
     * keep same.
     */
    stride = GRALLOC_ALIGN(stride, 64);
    if (size != NULL)
    {
        *size = stride * height;
        printf("size:%d", *size);
    }
    if (byte_stride != NULL)
    {
        *byte_stride = stride;
    }
    if (pixel_stride != NULL)
    {
        *pixel_stride = stride / pixel_size;
    }
}

/*
 * Computes the strides and size for an YV12 buffer
 *
 * width                  Public known width of the buffer in pixels
 * height                 Public known height of the buffer in pixels
 *
 * pixel_stride     (out) stride of the buffer in pixels
 * byte_stride      (out) stride of the buffer in bytes
 * size             (out) size of the buffer in bytes
 * stride_alignment (in)  stride aligment value in bytes.
 */
static int get_yv12_stride_and_size(int width, int height, int *pixel_stride,
				     int *byte_stride, int *size,
				     int stride_alignment)
{
	int luma_stride;


	/* 4:2:0 formats must have buffers with even height and width as the clump size is 2x2 pixels.
	 * Width will be even stride aligned anyway so just adjust height here for size calculation. */
	height = GRALLOC_ALIGN(height, 2);

	luma_stride = GRALLOC_ALIGN(width, stride_alignment);

	if (size != NULL)
	{
		int chroma_stride = GRALLOC_ALIGN(luma_stride / 2, stride_alignment);
		/* Simplification of ((height * luma_stride ) + 2 * ((height / 2) * chroma_stride)). */
		*size = height *(luma_stride + chroma_stride);
	}

	if (byte_stride != NULL)
	{
		*byte_stride = luma_stride;
	}

	if (pixel_stride != NULL)
	{
		*pixel_stride = luma_stride;
	}

	return 1;
}
static struct rockchip_bo *rockchip_drm_gem_alloc(struct rockchip_device *drm_dev, struct rockchip_drm_handle_t *handle)
{
    int width, height, format, pixel_size, pixel_stride, byte_stride, size;
    struct rockchip_bo *bo;
    uint32_t gem_handle;
    int ret = 0;

    width = handle->width;
    height = handle->height;
    format = handle->format;

    switch (format) {
    case DRM_FORMAT_XBGR8888:
    case DRM_FORMAT_XRGB8888:
    case DRM_FORMAT_RGBX8888:
    case DRM_FORMAT_BGRX8888:
    case DRM_FORMAT_ARGB8888:
    case DRM_FORMAT_ABGR8888:
    case DRM_FORMAT_RGBA8888:
    case DRM_FORMAT_BGRA8888:
        pixel_size = 4;
        break;
    case DRM_FORMAT_RGB888:
    case DRM_FORMAT_BGR888:
        pixel_size = 3;
        break;
    case DRM_FORMAT_RGB565:
    case DRM_FORMAT_BGR565:
        pixel_size = 2;
        break;
    case DRM_FORMAT_RGB332:
    case DRM_FORMAT_BGR233:
        pixel_size = 1;
        break;
	case DRM_FORMAT_NV12:
		pixel_size = 2;
		break;
    default:
        pixel_size = 4;
        LOGE("unsupport format: %d", format);
        break;
    }

	if (format == DRM_FORMAT_NV12) {	
		int yv12_align = 16;

		get_yv12_stride_and_size(width, height, &pixel_stride, &byte_stride, &size, yv12_align);
	} else {
		get_rgb_stride_and_size(width, height, pixel_size, &pixel_stride, &byte_stride, &size);
	}  

    if (handle->prime_fd > 0) {
        ret = drmPrimeFDToHandle(drm_dev->fd, handle->prime_fd,
            &gem_handle);
        if (ret) {
            printf("failed to convert prime fd to handle %d ret=%d", handle->prime_fd, ret);
            goto err;
        }
        bo = rockchip_bo_from_handle(drm_dev, gem_handle,
                         handle->flag, size);
        if (!bo) {
            struct drm_gem_close args;

            printf("failed to wrap bo handle=%d size=%d", handle->prime_fd, size);
            memset(&args, 0, sizeof(args));
            args.handle = gem_handle;
            drmIoctl(drm_dev->fd, DRM_IOCTL_GEM_CLOSE, &args);
            return NULL;
        }
        bo->fd = handle->prime_fd;//dma buffer fd
    } else {
        bo = rockchip_bo_create(drm_dev, size, handle->flag);
        if (!bo) {
            printf("failed to allocate bo %dx%dx%dx%d", handle->height, pixel_stride, byte_stride, size);
        }
        handle->size = size;
        handle->byte_stride = byte_stride;
        handle->pixel_stride = pixel_stride;
        gem_handle = rockchip_bo_handle(bo);
        ret = drmPrimeHandleToFD(drm_dev->fd, gem_handle, 0,
                     (int*)&bo->fd);
        if (ret) {
           printf("failed to get prime fd %d", ret);
            goto err_unref;
        }
    }

    return bo;

err_unref:
    rockchip_bo_destroy(bo);
err:
    return NULL;
}

static int rockchip_drm_gem_free(struct rockchip_bo *bo)
{
    if (bo->fd > 0)
        close(bo->fd);
    bo->fd = -1;

    rockchip_bo_destroy(bo);

    return 0;
}

static int rockchip_drm_gem_map(struct rockchip_bo *bo)
{
    unsigned long i,j;
    unsigned long *base_addr = NULL;

    rockchip_bo_map(bo);
    base_addr = (unsigned long *)bo->vaddr;
    if (0) {//test
        for (i = 0; i < 50; i++) {
            for (j = 0; j < 720; j++) {
                if ((i < 10) || ((i >= 20) && (i < 30))) {
                    *(base_addr + i * 720 + j) = 0xff00;
                } else {
                    if (frame_num == 0) {
                     *(base_addr + i * 720 + j) = 0x00ff;
                    } else if (frame_num == 1){
                        *(base_addr + i * 720 + j) = 0xff0000;
                    } else {
                        *(base_addr + i * 720 + j) = 0x00f0f0;
                    }
                }
            }
        }
        frame_num++;
    }
    return 0;
}

static int rockchip_drm_gem_unmap(struct rockchip_bo *bo)
{
    /*
     * the munmap(bo->vaddr, bo->size) will be called at
     * rockchip_drm_gem_free() -> rockchip_bo_destroy()
     * so here do nothing.
     */
    return 0;
}

static int rockchip_drm_add_fb(struct rockchip_device *drm_dev, struct rockchip_buff_info *buf_info, struct rockchip_drm_handle_t *drm_handle,
                   int afbdc)
{
    uint32_t handles[4], pitches[4], offsets[4];
    int fb_id, ret;
    struct rockchip_bo *bo = buf_info->bo;

    handles[0] = bo->handle;
    pitches[0] = drm_handle->byte_stride;
    offsets[0] = 0;
	if (drm_handle->format == DRM_FORMAT_NV12) {
		handles[1] = bo->handle;
		pitches[1] = drm_handle->byte_stride;
		offsets[1] = buf_info->src_w* buf_info->src_h;
	}
    if (!afbdc) {
        ret = drmModeAddFB2(drm_dev->fd,
            drm_handle->width, drm_handle->height, drm_handle->format,
            handles, pitches, offsets, (uint32_t*)&fb_id, 0);
    } else {
        __u64 modifier[4];
        memset(modifier, 0, sizeof(modifier));
        modifier[0] = DRM_FORMAT_MOD_ARM_AFBC;
        ret = drmModeAddFB2_ext(drm_dev->fd,
            drm_handle->width, drm_handle->height, drm_handle->format,
            handles, pitches, offsets, modifier,
            (uint32_t*)&fb_id, DRM_MODE_FB_MODIFIERS);
    }
    if (ret) {
        LOGE("failed to create fb ret=%d:%d %d, format:%d, handles:%d, pitch:%d, offset:%d, fb id:%d",
            ret, drm_handle->width, drm_handle->height, drm_handle->format, handles[0],
            pitches[0], offsets[0], fb_id);
        return ret;
    }

    buf_info->fb_id = fb_id;

    return 0;
}

#define DRM_ATOMIC_ADD_CRTC_PROP(object_id, value) \
do { \
    ret = drmModeAtomicAddProperty(req, crtc->crtc_id, object_id, value); \
    if (0) \
        printf("add crtc prop: crtc id:%d, obj id:%d, value:%d\n", crtc->crtc_id, object_id, value); \
    if (ret < 0) \
        LOGE("Failed to add prop[%d] to [%d]", value, object_id); \
} while (0)

#define DRM_ATOMIC_ADD_CONN_PROP(object_id, value) \
do { \
    ret = drmModeAtomicAddProperty(req, connector->connector_id, object_id, value); \
    if (0) \
        printf("add conn prop: connec id:%d, obj id:%d, value:%d\n", connector->connector_id, object_id, value); \
    if (ret < 0) \
        LOGE("Failed to add prop[%d] to [%d]", value, object_id); \
} while (0)

#define DRM_ATOMIC_ADD_PLANE_PROP(plane, object_id, value) \
do { \
    ret = drmModeAtomicAddProperty(req, plane->plane_id, object_id, value); \
    if (0) \
        printf("add plane prop: plane id:%d, obj id:%d, value:%d\n", plane->plane_id, object_id, value); \
    if (ret < 0) \
        LOGE("Failed to add prop[%d] to [%d]", value, object_id); \
} while (0)

static int rockchip_drm_commit(DRMConfig *cfg, int buf_idx)
{
    drmModeAtomicReq *req = NULL;
    drmModePlanePtr plane;
    struct plane_prop *plane_prop;
    int ret = 0;
    uint32_t flags = 0;
    struct rockchip_device *drm_dev = cfg->drm_dev;
    struct rockchip_buff_info *buf_info = cfg->buf_infos;
    struct drm_planes *drm_planes = cfg->drm_planes;
    drmModeCrtc *crtc = cfg->crtc;
    struct crtc_prop *crtc_prop = &cfg->crtc_prop;
    drmModeConnector *connector = cfg->connector;
    struct conn_prop *conn_prop = &cfg->conn_prop;

    if (buf_idx < 0 || buf_idx >= DRM_MAX_BUFFER_NUM) {
        LOGE("buffer index invalid!");
        return -1;
    }

    req = drmModeAtomicAlloc();
    /* add crtc prop */
    if (crtc_prop->pdaf_type)
        DRM_ATOMIC_ADD_CRTC_PROP(crtc_prop->pdaf_type, cfg->pdaf_type);
    if (crtc_prop->work_mode)
        DRM_ATOMIC_ADD_CRTC_PROP(crtc_prop->work_mode, cfg->pdaf_work_mode);
    /* add connect prop */
    if (conn_prop->csi_tx_path)
        DRM_ATOMIC_ADD_CONN_PROP(conn_prop->csi_tx_path, cfg->vop_csi_path);

    /* add plane prop */
    plane = drm_planes[cfg->plane_index].plane;
    plane_prop = &drm_planes[cfg->plane_index].plane_prop;
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_id, crtc->crtc_id);
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->fb_id, buf_info[buf_idx].fb_id);
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_x, 0); /*src data position */
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_y, 0);
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_w, buf_info[buf_idx].src_w << 16); /*src data size */
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_h, buf_info[buf_idx].src_h << 16);
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_x, 0);    /* display position */
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_y, 0);
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_w, buf_info[buf_idx].src_w); /* display size */
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_h, buf_info[buf_idx].src_h);
    DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->zpos, 0); /*todo: zpos, from 0 to zpos_max */

    /* for multi plane test */
    if (0) {
        /* add plane prop */
        plane = drm_planes[cfg->plane_index+1].plane;
        plane_prop = &drm_planes[cfg->plane_index+1].plane_prop;
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_id, crtc->crtc_id);
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->fb_id, buf_info[buf_idx].fb_id);
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_x, 0); /*src data position */
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_y, 0);
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_w, buf_info[buf_idx].src_w << 16); /*src data size */
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->src_h, buf_info[buf_idx].src_h << 16);
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_x, 0);    /* display position */
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_y, 0);
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_w, buf_info[buf_idx].src_w); /* display size */
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->crtc_h, buf_info[buf_idx].src_h);
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->zpos, 1); /*todo: zpos, from 0 to zpos_max */
    }

    if (plane_prop->data_type)
        DRM_ATOMIC_ADD_PLANE_PROP(plane, plane_prop->data_type, buf_info[buf_idx].data_type);

    /*
     * commit one frame
     */
    //flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;
    ret = drmModeAtomicCommit(drm_dev->fd, req, flags, NULL);
    if (ret) {
        LOGE("atomic: couldn't commit new state: %s, ret:%d", strerror(errno), ret);
    }

    drmModeAtomicFree(req);

    return ret;
}

// MIPITXContext
MIPITXContext::MIPITXContext() : TransferContext(TransferType::MIPI_TX) {
    LOGD("MIPITXContext constructor");
    _initialized = true;
}

MIPITXContext::~MIPITXContext() {
    LOGD("MIPITXContext destructor");
    _initialized = false;
}

const std::vector<std::shared_ptr<TransferDevice> >& MIPITXContext::findDevices() {

    std::vector<std::shared_ptr<TransferDevice> >& devs = _devices;
    devs.clear();

    auto dev = std::make_shared<MIPITXDevice>();
    dev->setID(MIPI_TX_DEVICE_ID);
    dev->setName("rockchip-drm");
    devs.push_back(dev);

    return devs;
}

// MIPITXDevice
MIPITXDevice::MIPITXDevice() : TransferDevice(TransferType::MIPI_TX) {
    LOGD("MIPITXDevice constructor");
}

MIPITXDevice::~MIPITXDevice() {
    LOGD("MIPITXDevice destructor");
    close();
}

int MIPITXDevice::MipiRxStartCapturing(DRMConfig *cfg)
{
    int i;
    int rx_fd = cfg->mipi_rx_dev.fd;
    struct v4l2_plane planes[MIPI_RX_FMT_NUM_PLANES];

    for (i = 0; i < DRM_MAX_BUFFER_NUM ; i++) {
        std::shared_ptr<MIPITXBuffer> buffer = _buf_queue.front();
        struct v4l2_buffer buf;

        if (_buf_queue.empty()) {
            LOGE("MIPITX buffer queue is empty!");
            break;
        }

        MEMSET(buf);
        buf.type = MipiRxBufType;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.index = i;

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == MipiRxBufType) {
            buf.m.planes = planes;
            buf.m.planes->length = cfg->width*cfg->height* 3 / 2;
            buf.length = MIPI_RX_FMT_NUM_PLANES;            
            buf.m.planes->m.fd = buffer->getFD();
        }

        if (-1 == xioctl(rx_fd, VIDIOC_QBUF, &buf)) {
            errno_exit("VIDIOC_QBUF");
        }

        _buf_queue.pop();
        _buf_queue.push(buffer);
    }
    if (-1 == xioctl(rx_fd, VIDIOC_STREAMON, &MipiRxBufType))
        errno_exit("VIDIOC_STREAMON");

    return 0;
}

int MIPITXDevice::MipiRxStopCapturing(DRMConfig *cfg)
{
        enum v4l2_buf_type type;
        int rx_fd = cfg->mipi_rx_dev.fd;

        type = MipiRxBufType;
        if (-1 == xioctl(rx_fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF");
        return 0;
}

static void MipiRxInitMap(int rx_fd)
{
        struct v4l2_requestbuffers req;
        unsigned int n_buffers;

        memset(&req, 0, sizeof(req));

        req.count = DRM_MAX_BUFFER_NUM;
        req.type = MipiRxBufType;
        req.memory = V4L2_MEMORY_DMABUF;

        if (-1 == xioctl(rx_fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "does not support "
                                 "memory mapping\n");
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                fprintf(stderr, "Insufficient buffer memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                memset(&buf, 0, sizeof(buf));

                buf.type = MipiRxBufType;
                buf.memory = V4L2_MEMORY_DMABUF;
                buf.index = n_buffers;

                if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == MipiRxBufType) {
                    struct v4l2_plane planes[MIPI_RX_FMT_NUM_PLANES];
                    buf.m.planes = planes;
                    buf.length = MIPI_RX_FMT_NUM_PLANES;
                }
                if (-1 == xioctl(rx_fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");
        }

}

static void MipiRxInit(int rx_fd, DRMConfig *cfg)
{
        struct v4l2_capability cap;
        struct v4l2_format fmt;

        if (-1 == xioctl(rx_fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, " no V4L2 device\n");
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
                !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
            fprintf(stderr, "it is not a video capture device, capabilities: %x\n",
            		cap.capabilities);
                exit(EXIT_FAILURE);
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                fprintf(stderr, "does not support streaming i/o\n");
                exit(EXIT_FAILURE);
        }

        memset(&fmt, 0, sizeof(fmt));

        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
            MipiRxBufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
            MipiRxBufType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        fmt.type = MipiRxBufType;
        fmt.fmt.pix.width = cfg->width;
        fmt.fmt.pix.height = cfg->height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (-1 == xioctl(rx_fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");
}

static int MipiRxOpen(void)
{
	int rx_fd;

    rx_fd = open(MipiRxDevName, O_RDWR /* required */ /*| O_NONBLOCK*/, 0);

    if (-1 == rx_fd) {
       fprintf(stderr, "Cannot open '%s': %d, %s\n",
               MipiRxDevName, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    return rx_fd;
}

static void MipiRxClose(DRMConfig *cfg)
{
        if (-1 == close(cfg->mipi_rx_dev.fd))
                errno_exit("close");
}

int MIPITXDevice::open(const char *spec) {

    std::lock_guard<std::mutex> lock(_mutex);
    int mipi_rx_fd;

    if (_initialized) {
        LOGW("MIPITX device has been opened!");
        return 0;
    }

    DRMConfig *cfg = &_cfg;

    if (drmInitialize(cfg, cfg->width, cfg->height, cfg->format) < 0) {
        LOGE("drm initialize failed!");
        return -1;
    }

    for (int i = 0; i < DRM_MAX_BUFFER_NUM; i++) {
        std::shared_ptr<MIPITXBuffer> buffer = std::make_shared<MIPITXBuffer>(cfg->buf_infos[i].bo, i);
        _buf_queue.push(buffer);
    }

    /* mipi rx init */
    if (0) {//test
        isp_init();
    } else {
        mipi_rx_fd = MipiRxOpen();
        cfg->mipi_rx_dev.fd = mipi_rx_fd;
        MipiRxInit(mipi_rx_fd, cfg);
        MipiRxInitMap(mipi_rx_fd);
        MipiRxStartCapturing(cfg);
    }
    
    _initialized = true;
    return 0;
}

int MIPITXDevice::close() {

    std::lock_guard<std::mutex> lock(_mutex);
    DRMConfig *cfg = &_cfg;

    drmRelease(cfg);

    MipiRxStopCapturing(cfg);
    MipiRxClose(cfg);

    cfg->reset();

    _initialized = false;

    return 0;
}

int MIPITXDevice::setBufferInfo(int width, int height, int format) {

    DRMConfig *cfg = &_cfg;
    cfg->width = width;
    cfg->height = height;
    cfg->format = format;

    return 0;
}

int MIPITXDevice::drmInitialize(DRMConfig *cfg, int width, int height, int format) {

    LOGD("drmInitialize begin!\n");
    cfg->drm_fd = drmOpen("rockchip", NULL);
    if (cfg->drm_fd < 0) {
        LOGE("failed to open rockchip drm: %s", strerror(errno));
        return cfg->drm_fd;
    }

    cfg->drm_dev = rockchip_device_create(cfg->drm_fd);
    if (cfg->drm_dev == NULL) {
        LOGE("failed to create rockchip drm device!");
        drmRelease(cfg);
        return -1;
    }

    if (drmSetClientCap(cfg->drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1)) {
        LOGE("Failed to set atomic cap %s", strerror(errno));
        drmRelease(cfg);
        return -1;
    }

    if (drmSetClientCap(cfg->drm_fd, DRM_CLIENT_CAP_ATOMIC, 1)) {
        LOGE("Failed to set atomic cap %s", strerror(errno));
        drmRelease(cfg);
        return -1;
    }

    drmModeResPtr res = drmModeGetResources(cfg->drm_fd);
    if (!res) {
        LOGE("Failed to get resources: %s", strerror(errno));
        drmRelease(cfg);
        return -1;
    }

    /*
     * Found active crtc.
     */
    int i = 0;
    uint32_t j = 0;
    drmModeObjectPropertiesPtr props;
    drmModePropertyPtr prop;
    int found_crtc = 0;
    for (i = 0; i < res->count_crtcs; ++i) {
        cfg->crtc = drmModeGetCrtc(cfg->drm_fd, res->crtcs[i]);
        if (!cfg->crtc) {
            LOGE("Could not get crtc %u: %s", res->crtcs[i], strerror(errno));
            continue;
        }

        props = drmModeObjectGetProperties(cfg->drm_fd, cfg->crtc->crtc_id, DRM_MODE_OBJECT_CRTC);
        if (!props) {
            LOGE("failed to found props crtc[%d] %s", cfg->crtc->crtc_id, strerror(errno));
            continue;
        }
        for (j = 0; j < props->count_props; j++) {
            prop = drmModeGetProperty(cfg->drm_fd, props->props[j]);
            if (!strcmp(prop->name, "ACTIVE")) {
                cfg->crtc_prop.crtc_active = prop->prop_id;
            } else if (!strcmp(prop->name, "PDAF_TYPE")) {
                cfg->crtc_prop.pdaf_type = prop->prop_id;
            } else if (!strcmp(prop->name, "WORK_MODE")) {
                cfg->crtc_prop.work_mode = prop->prop_id;
            }
        }
        /* maybe we need to find some special crtc, here we use the first crtc */
        found_crtc = 1;
        if (found_crtc)
            break;
    }
    if (i == res->count_crtcs) {
        LOGE("failed to find usable crtc");
        drmRelease(cfg);
        return -1;
    }

    /*
     * Found active connect.
     */
    drmModeModeInfo *mode = NULL;
    int found_conn = 0;
    cfg->conn_ids = (uint32_t*)calloc(res->count_connectors, sizeof(uint32_t));
    for (i = 0; i < res->count_connectors; ++i) {
        cfg->connector = drmModeGetConnector(cfg->drm_fd, res->connectors[i]);

        mode = &cfg->connector->modes[0];
        cfg->conn_ids[i] = cfg->connector->connector_id;

        props = drmModeObjectGetProperties(cfg->drm_fd, cfg->connector->connector_id, DRM_MODE_OBJECT_CONNECTOR);
        if (!props) {
            LOGE("failed to found props connect[%d] %s", cfg->connector->connector_id, strerror(errno));
            continue;
        }
        for (j = 0; j < props->count_props; j++) {
            prop = drmModeGetProperty(cfg->drm_fd, props->props[j]);
            if (!strcmp(prop->name, "CRTC_ID")) {
                cfg->conn_prop.crtc_id = prop->prop_id;
            } else if (!strcmp(prop->name, "CSI-TX-PATH")) {
                cfg->conn_prop.csi_tx_path = prop->prop_id;
            }
        }
        /* maybe we need to find some special connect, here we use the first connect */
        found_conn = 1;
        if (found_conn)
            break;
    }
    if (i == res->count_connectors) {
        LOGE("failed to find usable connect");
        drmRelease(cfg);
        return -1;
    }

    /* get planes */
    struct plane_prop plane_prop;
    drmModePlanePtr plane;
    int zpos_max = INT_MAX;
    drmModePlaneResPtr plane_res = drmModeGetPlaneResources(cfg->drm_fd);
    cfg->drm_planes = (struct drm_planes *)calloc(plane_res->count_planes, sizeof(struct drm_planes));
    for (i = 0; i < (int)plane_res->count_planes; i++) {
        memset(&plane_prop, 0, sizeof(plane_prop));

        plane = drmModeGetPlane(cfg->drm_fd, plane_res->planes[i]);
        props = drmModeObjectGetProperties(cfg->drm_fd, plane->plane_id,
                           DRM_MODE_OBJECT_PLANE);
        if (!props) {
            LOGE("failed to found props plane[%d] %s", plane->plane_id, strerror(errno));
            drmRelease(cfg);
            return -1;
        }

        for (j = 0; j < props->count_props; j++) {
            prop = drmModeGetProperty(cfg->drm_fd, props->props[j]);
            if (!strcmp(prop->name, "CRTC_ID"))
                plane_prop.crtc_id = prop->prop_id;
            else if (!strcmp(prop->name, "FB_ID"))
                plane_prop.fb_id = prop->prop_id;
            else if (!strcmp(prop->name, "SRC_X"))
                plane_prop.src_x = prop->prop_id;
            else if (!strcmp(prop->name, "SRC_Y"))
                plane_prop.src_y = prop->prop_id;
            else if (!strcmp(prop->name, "SRC_W"))
                plane_prop.src_w = prop->prop_id;
            else if (!strcmp(prop->name, "SRC_H"))
                plane_prop.src_h = prop->prop_id;
            else if (!strcmp(prop->name, "CRTC_X"))
                plane_prop.crtc_x = prop->prop_id;
            else if (!strcmp(prop->name, "CRTC_Y"))
                plane_prop.crtc_y = prop->prop_id;
            else if (!strcmp(prop->name, "CRTC_W"))
                plane_prop.crtc_w = prop->prop_id;
            else if (!strcmp(prop->name, "CRTC_H"))
                plane_prop.crtc_h = prop->prop_id;
            else if (!strcmp(prop->name, "PDAF_DATA_TYPE"))
                plane_prop.data_type = prop->prop_id;
            else if (!strcmp(prop->name, "ZPOS")) {
                plane_prop.zpos = prop->prop_id;
                zpos_max = props->prop_values[j];
                LOGD("zpos max:%d", zpos_max);
            } else if (!strcmp(prop->name, "FEATURE")) {
                plane_prop.feature = prop->prop_id;
                cfg->drm_planes[i].support_scale = props->prop_values[j] & 0x1;
                LOGD("scale:0x%lx", props->prop_values[j]);
            } else
                continue;
        }

        cfg->drm_planes[i].plane = plane;
        memcpy(&cfg->drm_planes[i].plane_prop, &plane_prop, sizeof(plane_prop));
 
    }

    /* get the first plane support scale */
    for (i = 0; i < (int)plane_res->count_planes; i++) {
        if (cfg->drm_planes[i].support_scale)
        {
            plane = cfg->drm_planes[i].plane;
            memcpy(&plane_prop, &cfg->drm_planes[i].plane_prop, sizeof(plane_prop));
            cfg->plane_index = i;//default use the first plane
            break;
        }
    }
    cfg->plane_num = (int)plane_res->count_planes;
    if (i == (int)plane_res->count_planes) {
        LOGE("can't find correct plane");
    }

    drmAllocBuffers(cfg, width, height, format);

    /* enable crtc and connect */
    struct rockchip_buff_info *buf_info = &_cfg.buf_infos[0];
    int ret = drmModeSetCrtc(cfg->drm_fd, cfg->crtc->crtc_id, buf_info->fb_id, 0, 0, cfg->conn_ids, res->count_connectors, mode);
    if (ret) {
        LOGE("drmModeSetCrtc faild:%d", ret);
    }

    LOGD("drmInitialize finished\n");
    return 0;
}

int MIPITXDevice::drmRelease(DRMConfig *cfg) {

 LOGE("drmRelease!\n");
    drmFreeBuffers(cfg);

    if (cfg->conn_ids != NULL) {
        free(cfg->conn_ids);
        cfg->conn_ids = NULL;
    }

    if (cfg->drm_planes != NULL) {
        free(cfg->drm_planes);
        cfg->drm_planes = NULL;
    }

    if (cfg->drm_dev != NULL) {
        rockchip_device_destroy(cfg->drm_dev);
        cfg->drm_dev = NULL;
    }

    if (cfg->drm_fd > 0) {
        drmClose(cfg->drm_fd);
        cfg->drm_fd = -1;
    }

    cfg->reset();

    return 0;
}

int MIPITXDevice::drmAllocBuffers(DRMConfig *cfg, int width, int height, int format) {


    for (int i = 0; i < DRM_MAX_BUFFER_NUM; i++) {
        struct rockchip_drm_handle_t *drm_handle = &cfg->drm_handles[i];
        struct rockchip_buff_info *buf_info = &cfg->buf_infos[i];

        drm_handle->format = format;
        drm_handle->width = width;
        drm_handle->height = height;
        drm_handle->afbc = 0;
        drm_handle->prime_fd = -1;
        drm_handle->flag = 0;
        drm_handle->file = NULL;

        buf_info->bo = rockchip_drm_gem_alloc(cfg->drm_dev, drm_handle);
        buf_info->src_w = drm_handle->width;
        buf_info->src_h = drm_handle->height;    
        rockchip_drm_gem_map(buf_info->bo);
        rockchip_drm_add_fb(cfg->drm_dev, buf_info, drm_handle, drm_handle->afbc);

    }
    return 0;
}

int MIPITXDevice::drmFreeBuffers(DRMConfig *cfg) {
    for (int i = 0; i < DRM_MAX_BUFFER_NUM; i++) {
        struct rockchip_buff_info *buf_info = &cfg->buf_infos[i];
        if (buf_info->fb_id > 0) {
            drmModeRmFB(cfg->drm_fd, buf_info->fb_id);
            rockchip_drm_gem_free(buf_info->bo);
        }
    }

    return 0;
}

int MIPITXDevice::recv(std::shared_ptr<TransferBuffer>& buffer) {
    std::shared_ptr<MIPITXBuffer> buf = NULL;
    int ret = 0;
    while (1) {
        ret = dequeueBuffer(buf);
        if (ret) {
            usleep(1000);
        } else {
            break;
        }
    }
    //LOGD("MIPITXDevice::recv!,buf id:%d, fd:%d, buf:%p",
    //    buf->getIndex(), buf->getFD(), buf);
    push(buf);

    return 0;
}

int MIPITXDevice::send(std::shared_ptr<TransferBuffer>& buffer) {
    DRMConfig *cfg = &_cfg;
    std::shared_ptr<MIPITXBuffer> buf = NULL;

    pop(buf);
    //LOGD("MIPITXDevice::send!,buf id:%d, fd:%d, buf:%p",
    //     buf->getIndex(), buf->getFD(), buf);
    rockchip_drm_commit(cfg, buf->getIndex());
    queueBuffer(buf);

    return 0;
}

int MIPITXDevice::dequeueBuffer(std::shared_ptr<MIPITXBuffer>& buffer) {

    std::lock_guard<std::mutex> lock(_mutex);
    DRMConfig *cfg = &_cfg;
    struct v4l2_buffer buf;
    int rx_fd = cfg->mipi_rx_dev.fd;

    MEMSET(buf);
    if (_buf_queue.empty()) {
        LOGE("MIPITX buffer queue is empty!");
        return -1;
    }

    buffer = _buf_queue.front();
    /* rx dequeue */
    buf.type = MipiRxBufType;
    buf.memory = V4L2_MEMORY_DMABUF;    
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == MipiRxBufType) {
        struct v4l2_plane planes[MIPI_RX_FMT_NUM_PLANES];
        buf.m.planes = planes;
        buf.length = MIPI_RX_FMT_NUM_PLANES;
    }

    if (-1 == xioctl(rx_fd, VIDIOC_DQBUF, &buf))
            errno_exit("VIDIOC_DQBUF");

    _buf_queue.pop();

    return 0;
}

int MIPITXDevice::queueBuffer(std::shared_ptr<MIPITXBuffer>& buffer) {

    std::lock_guard<std::mutex> lock(_mutex);
 
    DRMConfig *cfg = &_cfg;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[MIPI_RX_FMT_NUM_PLANES];
    int rx_fd = cfg->mipi_rx_dev.fd;

    MEMSET(buf);
    MEMSET(*planes);
    /*MIPI RX */
    buf.type = MipiRxBufType;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = buffer->getIndex();
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == MipiRxBufType) {
        buf.m.planes = planes;
        buf.m.planes->length = cfg->width*cfg->height* 3 / 2;
        buf.length = MIPI_RX_FMT_NUM_PLANES;
        buf.m.planes->m.fd = buffer->getFD();
    }

    if (-1 == xioctl(rx_fd, VIDIOC_QBUF, &buf)) {
        fprintf(stderr, "%s[%d]\n", __func__,__LINE__);
        errno_exit("VIDIOC_QBUF");
    }
    _buf_queue.push(buffer);
    if (_buf_queue.size() > DRM_MAX_BUFFER_NUM) {
        LOGE("MIPITX buffer queue size is invalid!");
        return -1;
    }
    return 0;
}

int MIPITXDevice::push(std::shared_ptr<MIPITXBuffer>& buffer) {

    std::pair<int, std::shared_ptr<TransferBuffer> > id_buffer;
    id_buffer.first = buffer->getFD();
    id_buffer.second = buffer;
    if (!_data_buf_queue.push(id_buffer)) {
        LOGE("data buffer push error!");
        return -1;
    }

    return 0;
}

int MIPITXDevice::pop(std::shared_ptr<MIPITXBuffer>& buffer) {

    std::pair<int, std::shared_ptr<TransferBuffer> > id_buffer;
    while (!_data_buf_queue.pop(id_buffer)) {
        usleep(1000);
    }
    buffer = std::static_pointer_cast<MIPITXBuffer>(id_buffer.second);

    return 0;
}

// MIPITXBuffer
MIPITXBuffer::MIPITXBuffer(struct rockchip_bo *bo, int buf_idx) : _bo(bo), _buf_idx(buf_idx) {
    map();
}

MIPITXBuffer::~MIPITXBuffer() {
    unmap();
}

void *MIPITXBuffer::map() {

    if (_virt_ptr != NULL && _size > 0) return _virt_ptr;

    struct rockchip_bo *bo = _bo;
    if (bo == NULL) return NULL;

    //rockchip_drm_gem_map(bo);

    _virt_ptr = bo->vaddr;
    _size = bo->size;

    return bo->vaddr;
}

int MIPITXBuffer::unmap() {

    struct rockchip_bo *bo = _bo;
    if (bo == NULL) return -1;
    rockchip_drm_gem_unmap(bo);

    return 0;
}

} // namespace RK
