/***********************************************************************************
 * Copyright (c) 2008, runyee Co.,Ltd.
 * All rights reserved.
 *
 * FileName : custom_key.c
 * Feature  : Single-key driver with GPIO and IRQ handling
 * Version  : V1.0
 *
 * History  :
 *  2025-08-26 by zgm
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
#include <linux/sysfs.h>

#define WAKEUP_HOLD_MS  3000

struct key_config {
    int gpio;
    int irq;
    int key_code;
    bool wakeup_enable;
    bool pressed;
    char name[32];
    struct device_attribute dev_attr;
};

struct gpio_keys_data {
    struct input_dev *input_dev;
    struct wakeup_source *ws;
    struct delayed_work sleep_work;
    struct key_config *keys;
    int key_count;
    struct mutex lock;
    struct device *dev;
    struct attribute_group attr_group;
    struct attribute **attrs;
};
struct gpio_keys_data *gData = NULL;

static int gpio_value = 1;
static ssize_t key_state_show(struct device *dev, 
                             struct device_attribute *attr, 
                             char *buf) {
    struct key_config *key = container_of(attr, struct key_config, dev_attr);
    struct gpio_keys_data *data = dev_get_drvdata(dev);
    int value;

    mutex_lock(&data->lock);
    value = gpio_get_value(key->gpio);
    key->pressed = (value ? 0 : 1);
    mutex_unlock(&data->lock);
    
    return sprintf(buf, "%d\n", key->pressed);
}

static void delayed_sleep_work_fn(struct work_struct *work) {
    struct gpio_keys_data *data = container_of(to_delayed_work(work), 
                                             struct gpio_keys_data, sleep_work);

    if(gpio_value)
	__pm_relax(data->ws);

    printk(KERN_INFO "Wake lock released after %d ms gpio_value:%d\n", WAKEUP_HOLD_MS,gpio_value);
}

static irqreturn_t key_irq_handler(int irq, void *dev_id) {
    struct gpio_keys_data *data = gData;
    struct key_config *key = (struct key_config *)dev_id;
    int value;

    if (!data || !key) {
        printk(KERN_ERR "Data or key structure not initialized\n");
        return IRQ_NONE;
    }

    cancel_delayed_work_sync(&data->sleep_work);
    __pm_stay_awake(data->ws);

    value = gpio_get_value(key->gpio);
    gpio_value = value;
    mutex_lock(&data->lock);
    key->pressed = (value ? 0 : 1);
    mutex_unlock(&data->lock);
    
    input_report_key(data->input_dev, key->key_code, key->pressed);
    input_sync(data->input_dev);
    printk(KERN_INFO "Key %s (GPIO %d) triggered, pressed: %d\n", 
           key->name, key->gpio, key->pressed);

    schedule_delayed_work(&data->sleep_work, msecs_to_jiffies(WAKEUP_HOLD_MS));
    return IRQ_HANDLED;
}

static const struct of_device_id custom_keys_of_match[] = {
    { .compatible = "mediatek,custom_keys" },
    {}
};
MODULE_DEVICE_TABLE(of, custom_keys_of_match);

static int custom_keys_suspend(struct device *dev) {
    struct gpio_keys_data *data = dev_get_drvdata(dev);
    int i;

    for (i = 0; i < data->key_count; i++) {
        if (data->keys[i].wakeup_enable) {
            enable_irq_wake(data->keys[i].irq);
            printk(KERN_INFO "Enabled wakeup on GPIO %d (key %s)\n", 
                  data->keys[i].gpio, data->keys[i].name);
        } else {
            disable_irq(data->keys[i].irq);
        }
    }
    return 0;
}

static int custom_keys_resume(struct device *dev) {
    struct gpio_keys_data *data = dev_get_drvdata(dev);
    int i;

    for (i = 0; i < data->key_count; i++) {
        if (data->keys[i].wakeup_enable) {
            disable_irq_wake(data->keys[i].irq);
        } else {
            enable_irq(data->keys[i].irq);
        }
    }
    printk(KERN_INFO "Restored GPIO interrupts after resume\n");
    return 0;
}

static const struct dev_pm_ops custom_keys_pm_ops = {
    .suspend = custom_keys_suspend,
    .resume = custom_keys_resume,
};

static int custom_keys_probe(struct platform_device *pdev) {
    struct gpio_keys_data *data;
    struct device_node *np = pdev->dev.of_node;
    struct device_node *child;
    int count = 0, ret, i = 0;
    const char *key_name;
    for_each_child_of_node(np, child)
        count++;

    if (count == 0) {
        dev_err(&pdev->dev, "No keys defined in device tree\n");
        return -EINVAL;
    }

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->key_count = count;
    data->keys = devm_kzalloc(&pdev->dev, count * sizeof(struct key_config), GFP_KERNEL);
    if (!data->keys)
        return -ENOMEM;

    data->attrs = devm_kzalloc(&pdev->dev, (count + 1) * sizeof(struct attribute *), GFP_KERNEL);
    if (!data->attrs)
        return -ENOMEM;

    data->dev = &pdev->dev;
    mutex_init(&data->lock);

    for_each_child_of_node(np, child) {
        struct key_config *key = &data->keys[i];
        if (of_property_read_string(child, "label", &key_name)) {
            snprintf(key->name, sizeof(key->name), "key%d", i);
        } else {
            snprintf(key->name, sizeof(key->name), "%s", key_name);
        }
        key->gpio = of_get_named_gpio(child, "gpios", 0);
        if (key->gpio < 0) {
            dev_err(&pdev->dev, "Failed to get GPIO from DT for key %s\n", key->name);
            return -EINVAL;
        }
        
        if (!gpio_is_valid(key->gpio)) {
            dev_err(&pdev->dev, "Invalid GPIO number %d for key %s\n", key->gpio, key->name);
            return -EINVAL;
        }
        
        ret = devm_gpio_request(&pdev->dev, key->gpio, key->name);
        if (ret) {
            dev_err(&pdev->dev, "Failed to request GPIO %d for key %s\n", key->gpio, key->name);
            return ret;
        }
        
        gpio_direction_input(key->gpio);
        
        key->pressed = (gpio_get_value(key->gpio) ? 0 : 1);
        printk(KERN_INFO "Boot GPIO %d (%s) initial state: %d (pressed: %d)\n",
               key->gpio, key->name, gpio_get_value(key->gpio), key->pressed);
        key->irq = gpio_to_irq(key->gpio);
        if (key->irq < 0) {
            dev_err(&pdev->dev, "Failed to get IRQ for GPIO %d (key %s)\n", key->gpio, key->name);
            return key->irq;
        }
        
        if (of_property_read_u32(child, "linux,code", &key->key_code)) {
            dev_err(&pdev->dev, "No key code specified for GPIO %d\n", key->gpio);
            return -EINVAL;
        }
        
        key->wakeup_enable = of_property_read_bool(child, "wakeup-source");
        sysfs_attr_init(&key->dev_attr.attr);
        key->dev_attr.attr.name = key->name;
        key->dev_attr.attr.mode = S_IRUGO;
        key->dev_attr.show = key_state_show;
        key->dev_attr.store = NULL;
        data->attrs[i] = &key->dev_attr.attr;
        
        printk(KERN_INFO "Configured key %s: GPIO=%d, code=%d, wakeup=%d\n", 
               key->name, key->gpio, key->key_code, key->wakeup_enable);
        i++;
    }
    data->attrs[count] = NULL;
    data->attr_group.attrs = data->attrs;

    data->input_dev = devm_input_allocate_device(&pdev->dev);
    if (!data->input_dev) {
        dev_err(&pdev->dev, "Failed to allocate input device\n");
        return -ENOMEM;
    }
    data->input_dev->name = "custom_keys_input";
    __set_bit(EV_KEY, data->input_dev->evbit);
    for (i = 0; i < data->key_count; i++)
        __set_bit(data->keys[i].key_code, data->input_dev->keybit);
    
    ret = input_register_device(data->input_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register input device: %d\n", ret);
        return ret;
    }

    for (i = 0; i < data->key_count; i++) {
        ret = devm_request_irq(&pdev->dev, data->keys[i].irq, key_irq_handler,
                              IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                              data->keys[i].name, &data->keys[i]);
        if (ret) {
            dev_err(&pdev->dev, "Failed to request IRQ %d for key %s\n", 
                   data->keys[i].irq, data->keys[i].name);
            return ret;
        }
    }

    data->ws = wakeup_source_register(&pdev->dev, "custom_keys_ws");
    if (!data->ws) {
        dev_err(&pdev->dev, "Failed to register wakeup source\n");
        return -ENOMEM;
    }

    INIT_DELAYED_WORK(&data->sleep_work, delayed_sleep_work_fn);

    ret = device_init_wakeup(&pdev->dev, true);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable wakeup: %d\n", ret);
        goto err_unregister_ws;
    }
    ret = sysfs_create_group(&pdev->dev.kobj, &data->attr_group);
    if (ret) {
        dev_err(&pdev->dev, "Failed to create sysfs group: %d\n", ret);
        goto err_unregister_ws;
    }
    gData = data;
    platform_set_drvdata(pdev, data);
    printk(KERN_INFO "Custom keys driver probed successfully, %d keys configured\n", 
           data->key_count);
    return 0;

err_unregister_ws:
    wakeup_source_unregister(data->ws);
    return ret;
}

static int custom_keys_remove(struct platform_device *pdev) {
    struct gpio_keys_data *data = platform_get_drvdata(pdev);
    cancel_delayed_work_sync(&data->sleep_work);
    wakeup_source_unregister(data->ws);
    input_unregister_device(data->input_dev);
    sysfs_remove_group(&pdev->dev.kobj, &data->attr_group);
    mutex_destroy(&data->lock);
    printk(KERN_INFO "Custom keys driver removed\n");
    return 0;
}

static struct platform_driver custom_keys_driver = {
    .probe = custom_keys_probe,
    .remove = custom_keys_remove,
    .driver = {
        .name = "custom_keys",
        .of_match_table = custom_keys_of_match,
        .pm = &custom_keys_pm_ops,
        .owner = THIS_MODULE,
    },
};
module_platform_driver(custom_keys_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zgm@runyee.com.cn");
MODULE_DESCRIPTION("Custom key driver with GPIO and wakeup support");

