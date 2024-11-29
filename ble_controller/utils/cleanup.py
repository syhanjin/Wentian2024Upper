# -*- coding: utf-8 -*-
import asyncio
import atexit

from .color import *

_cleanup_list_sync = []
_cleanup_list_async = []


def cleanup(func):
    if asyncio.iscoroutinefunction(func):
        async def wrapped_func(*args, **kwargs):
            print(blue(f'Running Cleanup(async): {func.__name__}'))
            try:
                return await func(*args, **kwargs)
            except Exception as e:
                print(red(e))

        _cleanup_list_async.append(wrapped_func)
    else:
        def wrapped_func(*args, **kwargs):
            print(blue(f'Running Cleanup(sync): {func.__name__}'))
            try:
                return func(*args, **kwargs)
            except Exception as e:
                print(red(e))

        _cleanup_list_sync.append(wrapped_func)
    return func


register_cleanup = cleanup


def _perform_cleanup():
    """
    没什么diao用，放弃挣扎
    :return:
    """
    print('Cleaning up...')
    for func in _cleanup_list_sync:
        func()
    # 获取或创建事件循环
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        # 如果没有运行中的事件循环，则创建新的事件循环
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

    # 执行异步清理
    try:
        loop.run_until_complete(asyncio.gather(*[func() for func in _cleanup_list_async]))
    finally:
        # 关闭事件循环
        loop.close()


atexit.register(_perform_cleanup)
