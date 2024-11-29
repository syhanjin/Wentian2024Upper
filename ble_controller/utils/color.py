# color.py
from colorama import Back, Fore, init

init(autoreset=True)


def red(s):
    """
    前景色:红色  背景色:默认
    """
    return Fore.RED + str(s) + Fore.RESET


def green(s):
    """
    前景色:绿色  背景色:默认
    """
    return Fore.GREEN + str(s) + Fore.RESET


def yellow(s):
    """
    前景色:黄色  背景色:默认
    """
    return Fore.YELLOW + str(s) + Fore.RESET


#


def blue(s):
    """
    前景色:蓝色  背景色:默认
    """
    return Fore.BLUE + str(s) + Fore.RESET


#


def magenta(s):
    """
    前景色:洋红色  背景色:默认
    """
    return Fore.MAGENTA + str(s) + Fore.RESET


#


def cyan(s):
    """
    前景色:青色  背景色:默认
    """
    return Fore.CYAN + str(s) + Fore.RESET


def white(s):
    """
    前景色:白色  背景色:默认
    """
    return Fore.WHITE + str(s) + Fore.RESET


def black(s):
    """
    前景色:黑色  背景色:默认
    """
    return Fore.BLACK


def white_green(s):
    """
    前景色:白色  背景色:绿色
    """
    return Fore.WHITE + Back.GREEN + str(s) + Fore.RESET + Back.RESET
