/***********************************************************************************
 * Copyright (c) 2008, runyee Co.,Ltd.
 * All rights reserved.
 *
 * FileName : custom_key.c
 * Version  : V1.0
 *
 * History  :
 *  2025-06-03 by zgm
 ***********************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include "custom-toggle.h"

#define DRIVER_NAME     "custom-toggle"
#define DEVICE_NAME     "camera_toggle"
#define CAMERA_LOCK_DEV "camera_lock"

#define MAX_CAMERAS     3
#define POWER_DELAY     50  /* ms */
#define SWITCH_CHECK_INTERVAL 100

#define CAMERA_LOCK_MAGIC     'C'
#define CAMERA_LOCK_GET_STATE _IOR(CAMERA_LOCK_MAGIC, 0, int)

struct camera_pmic_config {
    int pnd_gpio;
    struct regulator *ldo;
};

struct toggle_switch_data {
    struct input_dev *input_dev;
    struct wakeup_source *ws;
    struct delayed_work camera_work;
    
    int toggle_gpio;
    int irq;
    
    int toggle_state;
    bool camera_locked;
    
    struct camera_pmic_config cameras[MAX_CAMERAS];
    
    struct mutex lock;
    struct cdev cdev;
    struct class *class;
    dev_t devt;
    
    struct list_head camera_list;
    struct mutex camera_list_lock;
};

static struct toggle_switch_data *g_toggle_data = NULL;

static void camera_control_work(struct work_struct *work) {
    struct toggle_switch_data *data = container_of(work, 
                                                  struct toggle_switch_data, 
                                                  camera_work.work);
    struct camera_device *cam, *tmp;
    
    mutex_lock(&data->lock);
    
    printk(KERN_INFO "%s: Camera control requested, state: %s\n", 
           DRIVER_NAME, data->toggle_state ? "ON" : "OFF");

    if (!data->toggle_state) {
        mutex_lock(&data->camera_list_lock);
        list_for_each_entry_safe(cam, tmp, &data->camera_list, list) {
            if (cam->power_off) {
                printk(KERN_INFO "%s: Notifying camera to power off\n", DRIVER_NAME);
                //cam->power_off(cam->priv);
            }
        }
        mutex_unlock(&data->camera_list_lock);
    }

    data->camera_locked = !data->toggle_state;
    
    mutex_unlock(&data->lock);
}

static irqreturn_t toggle_irq_handler(int irq, void *dev_id) {
    struct toggle_switch_data *data = dev_id;
    int state;
    
    state = gpio_get_value(data->toggle_gpio);
    
    if (state != data->toggle_state) {
        data->toggle_state = state;

        input_report_key(data->input_dev, KEY_CAMERA, data->toggle_state);
        input_sync(data->input_dev);
        
        printk(KERN_INFO "%s: Toggle switch state changed to %s\n", 
               DRIVER_NAME, data->toggle_state ? "ON" : "OFF");

        schedule_delayed_work(&data->camera_work, 0);
    }
    
    return IRQ_HANDLED;
}

static int toggle_suspend(struct device *dev) {
    struct toggle_switch_data *data = dev_get_drvdata(dev);
    
    enable_irq_wake(data->irq);
    printk(KERN_INFO "%s: Enabled wakeup on GPIO interrupt\n", DRIVER_NAME);
    
    return 0;
}

static int toggle_resume(struct device *dev) {
    struct toggle_switch_data *data = dev_get_drvdata(dev);
    
    disable_irq_wake(data->irq);
    printk(KERN_INFO "%s: Restored GPIO interrupt after resume\n", DRIVER_NAME);
    
    return 0;
}

static const struct dev_pm_ops toggle_pm_ops = {
    .suspend = toggle_suspend,
    .resume = toggle_resume,
};

bool is_camera_locked(void) {
    //if (g_toggle_data)
    //    return g_toggle_data->camera_locked;
    return false;
}
EXPORT_SYMBOL(is_camera_locked);

static int camera_switch_monitor(void *data) {
    struct camera_device *cam = (struct camera_device *)data;
    struct toggle_switch_data *toggle_data = g_toggle_data;
    
    printk(KERN_INFO "%s: Camera switch monitor thread started\n", DRIVER_NAME);
    
    cam->thread_running = true;
    
    while (!kthread_should_stop()) {
        if (toggle_data && !toggle_data->toggle_state) {
            printk(KERN_INFO "%s: Toggle switch is OFF, closing camera\n", DRIVER_NAME);
            if (cam->power_off) {
                //cam->power_off(cam->priv);
            }
            break;
        }

        msleep(SWITCH_CHECK_INTERVAL);
    }
    
    cam->thread_running = false;
    printk(KERN_INFO "%s: Camera switch monitor thread stopped\n", DRIVER_NAME);
    
    return 0;
}

int register_camera_device(struct camera_device *cam) {
    struct toggle_switch_data *data = g_toggle_data;
    
    if (!data)
        return -ENODEV;
        
    mutex_lock(&data->camera_list_lock);
    list_add_tail(&cam->list, &data->camera_list);
    mutex_unlock(&data->camera_list_lock);
    
    printk(KERN_INFO "%s: Camera device registered\n", DRIVER_NAME);

    if (!data->toggle_state) {
        cam->monitor_thread = kthread_run(camera_switch_monitor, cam, "camera_switch_monitor");
        if (IS_ERR(cam->monitor_thread)) {
            printk(KERN_ERR "%s: Failed to start camera switch monitor thread\n", DRIVER_NAME);
            return PTR_ERR(cam->monitor_thread);
        }
    }
    
    return 0;
}
EXPORT_SYMBOL(register_camera_device);

int unregister_camera_device(struct camera_device *cam) {
    struct toggle_switch_data *data = g_toggle_data;
    
    if (!data)
        return -ENODEV;

    if (cam->monitor_thread && cam->thread_running) {
        kthread_stop(cam->monitor_thread);
        cam->monitor_thread = NULL;
    }
    
    mutex_lock(&data->camera_list_lock);
    list_del(&cam->list);
    mutex_unlock(&data->camera_list_lock);
    
    printk(KERN_INFO "%s: Camera device unregistered\n", DRIVER_NAME);
    return 0;
}
EXPORT_SYMBOL(unregister_camera_device);

static int camera_lock_open(struct inode *inode, struct file *filp) {
    filp->private_data = g_toggle_data;
    return 0;
}

static int camera_lock_release(struct inode *inode, struct file *filp) {
    return 0;
}

static long camera_lock_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct toggle_switch_data *data = filp->private_data;
    int ret = 0;
    int state;

    switch (cmd) {
        case CAMERA_LOCK_GET_STATE:
            state = data->camera_locked;
            ret = copy_to_user((int *)arg, &state, sizeof(int));
            break;
            
        default:
            ret = -ENOTTY;
    }

    return ret;
}

static const struct file_operations camera_lock_fops = {
    .owner = THIS_MODULE,
    .open = camera_lock_open,
    .release = camera_lock_release,
    .unlocked_ioctl = camera_lock_ioctl,
};

static int toggle_probe(struct platform_device *pdev) {
    struct toggle_switch_data *data;
    struct device_node *np = pdev->dev.of_node;
    struct device_node *child_node;
    int ret, i;
    enum of_gpio_flags flags;
    const char *ldo_name;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        dev_err(&pdev->dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }
    
    platform_set_drvdata(pdev, data);
    g_toggle_data = data;

    mutex_init(&data->lock);
    mutex_init(&data->camera_list_lock);

    data->toggle_gpio = of_get_named_gpio_flags(np, "gpios", 0, &flags);
    if (!gpio_is_valid(data->toggle_gpio)) {
        dev_err(&pdev->dev, "Invalid toggle switch GPIO\n");
        return -EINVAL;
    }
    
    ret = devm_gpio_request(&pdev->dev, data->toggle_gpio, DRIVER_NAME);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request toggle switch GPIO\n");
        return ret;
    }
    
    ret = gpio_direction_input(data->toggle_gpio);
    if (ret) {
        dev_err(&pdev->dev, "Failed to set toggle switch GPIO direction\n");
        return ret;
    }

    data->toggle_state = gpio_get_value(data->toggle_gpio);
    printk(KERN_INFO "%s: Toggle switch initial state: %s\n", 
           DRIVER_NAME, data->toggle_state ? "ON" : "OFF");

    data->input_dev = devm_input_allocate_device(&pdev->dev);
    if (!data->input_dev) {
        dev_err(&pdev->dev, "Failed to allocate input device\n");
        return -ENOMEM;
    }
    
    data->input_dev->name = DEVICE_NAME;
    __set_bit(EV_KEY, data->input_dev->evbit);
    __set_bit(KEY_CAMERA, data->input_dev->keybit);
    
    ret = input_register_device(data->input_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register input device\n");
        return ret;
    }

    data->irq = gpio_to_irq(data->toggle_gpio);
    if (data->irq < 0) {
        dev_err(&pdev->dev, "Failed to get IRQ for GPIO\n");
        return data->irq;
    }
    
    ret = devm_request_irq(&pdev->dev, data->irq, toggle_irq_handler,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          DRIVER_NAME, data);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request IRQ\n");
        return ret;
    }

    data->ws = wakeup_source_register(&pdev->dev, DRIVER_NAME);
    if (!data->ws) {
        dev_err(&pdev->dev, "Failed to register wakeup source\n");
        return -ENOMEM;
    }
    
    device_init_wakeup(&pdev->dev, true);

    INIT_DELAYED_WORK(&data->camera_work, camera_control_work);

    INIT_LIST_HEAD(&data->camera_list);
#if 0
    i = 0;
    for_each_child_of_node(np, child_node) {
        if (of_device_is_compatible(child_node, "mediatek,pmic-ldo-cam0") ||
            of_device_is_compatible(child_node, "mediatek,pmic-ldo-cam1") ||
            of_device_is_compatible(child_node, "mediatek,pmic-ldo-cam2")) {

            data->cameras[i].pnd_gpio = of_get_named_gpio_flags(child_node, 
                                                              "gpio", 0, &flags);
            if (data->cameras[i].pnd_gpio >= 0) {
                ret = devm_gpio_request(&pdev->dev, data->cameras[i].pnd_gpio, 
                                       "camera_pnd");
                if (ret) {
                    dev_warn(&pdev->dev, "Failed to request camera PND GPIO\n");
                    data->cameras[i].pnd_gpio = -1;
                } else {
                    printk(KERN_INFO "%s: Camera %d PND GPIO: %d\n", 
                           DRIVER_NAME, i, data->cameras[i].pnd_gpio);
                }
            }
            
            if (of_property_read_string(child_node, "ldo-supply", &ldo_name) == 0) {
                data->cameras[i].ldo = regulator_get(&pdev->dev, ldo_name);
                if (IS_ERR(data->cameras[i].ldo)) {
                    dev_warn(&pdev->dev, "Failed to get camera LDO regulator\n");
                    data->cameras[i].ldo = NULL;
                } else {
                    printk(KERN_INFO "%s: Camera %d LDO regulator acquired: %s\n", 
                           DRIVER_NAME, i, ldo_name);
                }
            } else {
                dev_warn(&pdev->dev, "Failed to get camera LDO supply\n");
            }
            
            i++;
            if (i >= MAX_CAMERAS)
                break;
        }
    }
#endif
    ret = alloc_chrdev_region(&data->devt, 0, 1, CAMERA_LOCK_DEV);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate char device region\n");
        return ret;
    }
    
    cdev_init(&data->cdev, &camera_lock_fops);
    data->cdev.owner = THIS_MODULE;
    
    ret = cdev_add(&data->cdev, data->devt, 1);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add char device\n");
        unregister_chrdev_region(data->devt, 1);
        return ret;
    }
    
    data->class = class_create(THIS_MODULE, "camera_lock");
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create device class\n");
        cdev_del(&data->cdev);
        unregister_chrdev_region(data->devt, 1);
        return PTR_ERR(data->class);
    }
    
    device_create(data->class, NULL, data->devt, NULL, CAMERA_LOCK_DEV);

    data->camera_locked = !data->toggle_state;
    schedule_delayed_work(&data->camera_work, 0);
    
    printk(KERN_INFO "%s: Driver initialized successfully\n", DRIVER_NAME);
    return 0;
}

static int toggle_remove(struct platform_device *pdev) {
    struct toggle_switch_data *data = platform_get_drvdata(pdev);
    int i;

    cancel_delayed_work_sync(&data->camera_work);

    {
        struct camera_device *cam, *tmp;
        
        mutex_lock(&data->camera_list_lock);
        list_for_each_entry_safe(cam, tmp, &data->camera_list, list) {
            if (cam->monitor_thread && cam->thread_running) {
                kthread_stop(cam->monitor_thread);
                cam->monitor_thread = NULL;
            }
        }
        mutex_unlock(&data->camera_list_lock);
    }
#if 0
    for (i = 0; i < MAX_CAMERAS; i++) {
        if (data->cameras[i].ldo) {
            regulator_put(data->cameras[i].ldo);
        }
    }
#endif
    device_destroy(data->class, data->devt);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->devt, 1);

    wakeup_source_unregister(data->ws);
    
    printk(KERN_INFO "%s: Driver removed\n", DRIVER_NAME);
    g_toggle_data = NULL;
    return 0;
}

static const struct of_device_id toggle_of_match[] = {
    { .compatible = "mediatek,custom-toggle" },
    {},
};
MODULE_DEVICE_TABLE(of, toggle_of_match);

static struct platform_driver toggle_driver = {
    .probe = toggle_probe,
    .remove = toggle_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = toggle_of_match,
        .pm = &toggle_pm_ops,
        .owner = THIS_MODULE,
    },
};

module_platform_driver(toggle_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zgm@runyee.com.cn");
MODULE_DESCRIPTION("Custom key driver");

