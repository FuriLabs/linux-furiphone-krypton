#ifndef __CUSTOM_TOGGLE_H__
#define __CUSTOM_TOGGLE_H__

#include <linux/kthread.h>

struct camera_device {
    struct list_head list;
    void (*power_off)(void *priv);
    void *priv;
    struct task_struct *monitor_thread;
    bool thread_running;
};

bool is_camera_locked(void);
int register_camera_device(struct camera_device *cam);
int unregister_camera_device(struct camera_device *cam);


#endif
