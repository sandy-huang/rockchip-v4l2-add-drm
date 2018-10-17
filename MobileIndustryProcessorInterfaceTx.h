// by felix

#ifndef __MOBILE_INDUSTRY_PROCESSOR_INTERFACE_TX_H_
#define __MOBILE_INDUSTRY_PROCESSOR_INTERFACE_TX_H_

#include <string.h>

#include <queue>

#include "TransferDevice.h"
#include "TransferQueue.h"

extern "C" {
#include <drm_fourcc.h>
#include <rockchip/rockchip_drmif.h>
#include <drm.h>
#include <libdrm_macros.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
}

namespace RK {

#define DRM_MAX_BUFFER_NUM 4

enum csi_path_mode {
	VOP_PATH,
	BYPASS_PATH
};

enum vop_pdaf_mode {
	VOP_HOLD_MODE = 0,
	VOP_NORMAL_MODE,
	VOP_PINGPONG_MODE,
	VOP_BYPASS_MODE,
	VOP_BACKGROUND_MODE,
	VOP_ONEFRAME_MODE,
	VOP_ONEFRAME_NOSEND_MODE
};

enum vop_pdaf_type {
	VOP_PDAF_TYPE_DEFAULT = 0,
	VOP_PDAF_TYPE_HBLANK,
	VOP_PDAF_TYPE_VBLANK,
};

enum rockchip_pdaf_pos {
	ROCKCHIP_DRM_PDAF_POS_DEFAULT = 0,
	ROCKCHIP_DRM_PDAF_POS_BEFORE_VBLANK,
	ROCKCHIP_DRM_PDAF_POS_AFTER_VBLANK,
	ROCKCHIP_DRM_PDAF_POS_HBLANK,
	ROCKCHIP_DRM_PDAF_POS_MAX,
};

#define VOP_PDAF_VBLANK_POS0	ROCKCHIP_DRM_PDAF_POS_DEFAULT
#define VOP_PDAF_VBLANK_POS1	ROCKCHIP_DRM_PDAF_POS_BEFORE_VBLANK

struct crtc_prop {
    int crtc_active;
    int pdaf_type;
    int work_mode;
};

struct conn_prop {
    int crtc_id;
    int csi_tx_path;
};

struct rockchip_drm_handle_t {
    char *file; /* file path and name */
    int format; /* data format define at drm_fourcc.h */
    int width; /* pixel */
    int height; /* pixel */
    int afbc;  /* gpu compress format */

    /* file descriptors */
    int prime_fd; /* input fd, maybe from different process after dup */
    int flag;

    /* return value */
    int byte_stride;
    int size;
    int pixel_stride;
};

struct rockchip_buff_info {
    struct rockchip_bo *bo;
    int fb_id;

    int src_w;
    int src_h;

/* indicate pdaf pos */
    int pdaf_pos;
    int data_type;
};

struct DRMConfig {

    DRMConfig() {
        reset();
    }

    int reset() {

        width = 720;
        height = 1280;
        format = DRM_FORMAT_XBGR8888;//DRM_FORMAT_NV12;//DRM_FORMAT_XBGR8888;
        drm_fd = -1;

	pdaf_work_mode = VOP_NORMAL_MODE;
	pdaf_type = VOP_PDAF_TYPE_DEFAULT;
	vop_csi_path = VOP_PATH;

        drm_dev = NULL;
        conn_ids = NULL;
        drm_planes = NULL;
        crtc = NULL;
        memset(&crtc_prop, 0, sizeof(crtc_prop));
        connector = NULL;
        memset(&conn_prop, 0, sizeof(conn_prop));
        memset(&drm_handles[0], 0 , sizeof(drm_handles));
        memset(&buf_infos[0], 0, sizeof(buf_infos));
        plane_num = 1;
        plane_index = 0;

        return 0;
    }

    int width;
    int height;
    int format;
    int drm_fd;
    int pdaf_work_mode;
    int pdaf_type;
    int vop_csi_path;

    struct rockchip_device *drm_dev;
    uint32_t *conn_ids;
    struct drm_planes *drm_planes;
    drmModeCrtcPtr crtc;
    struct crtc_prop crtc_prop;
    drmModeConnectorPtr connector;
    struct conn_prop conn_prop;
    struct rockchip_drm_handle_t drm_handles[DRM_MAX_BUFFER_NUM];
    struct rockchip_buff_info buf_infos[DRM_MAX_BUFFER_NUM];

    struct rockchip_device mipi_rx_dev;

    int plane_num;
    int plane_index;
};

class MIPITXContext : public TransferContext {

public:

    MIPITXContext();
    virtual ~MIPITXContext();

    virtual const std::vector<std::shared_ptr<TransferDevice> >& findDevices();

protected:

private:

};

class MIPITXBuffer;

class MIPITXDevice : public TransferDevice {

public:

    MIPITXDevice();
    virtual ~MIPITXDevice();

    virtual int open(const char *spec = NULL);
    virtual int close();
    virtual int reset(const char *spec = NULL) { return 0; }

    virtual int send(std::shared_ptr<TransferBuffer>& buffer);
    virtual int recv(std::shared_ptr<TransferBuffer>& buffer);

    int setBufferInfo(int width, int height, int format);

private:

    // for DRM VOP-CSI TX
    int drmInitialize(DRMConfig *cfg, int width, int height, int format);
    int drmRelease(DRMConfig *cfg);
    int drmAllocBuffers(DRMConfig *cfg, int width, int height, int format);
    int drmFreeBuffers(DRMConfig *cfg);
    int dequeueBuffer(std::shared_ptr<MIPITXBuffer>& buffer);
    int queueBuffer(std::shared_ptr<MIPITXBuffer>& buffer);
    int push(std::shared_ptr<MIPITXBuffer>& buffer);
    int pop(std::shared_ptr<MIPITXBuffer>& buffer);

    //for CSI RX
    int MipiRxStartCapturing(DRMConfig *cfg);
    int MipiRxStopCapturing(DRMConfig *cfg);

    std::queue<std::shared_ptr<MIPITXBuffer> > _buf_queue;
    std::mutex _mutex;
    TransferQueue _data_buf_queue;

    DRMConfig _cfg;
};

class MIPITXBuffer : public TransferBuffer {

public:

    MIPITXBuffer(struct rockchip_bo *bo, int buf_idx);

    virtual ~MIPITXBuffer();

    int getIndex() { return _buf_idx; }
    int getFD() {
        if (_bo) return _bo->fd;
        return -1;
    }

private:
    virtual void *map();
    virtual int unmap();

    struct rockchip_bo *_bo;
    int _buf_idx;
};

} // namespace RK

#endif /* end of __MOBILE_INDUSTRY_PROCESSOR_INTERFACE_TX_H_ */
