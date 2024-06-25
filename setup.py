# 참고자료
# http://swdeveloper.tistory.com/34
# https://code.tutsplus.com/ko/tutorials/how-to-share-your-python-packages--cms-26114
# https://code.tutsplus.com/ko/tutorials/how-to-write-your-own-python-packages--cms-26076
from setuptools import setup, find_packages

setup(
    name = "CodingRider",
    version = "2.9",
    description = "Library for CodingRider",
    author = "ALUX",
    author_email = "devdrone@aluxonline.com",
    url = "https://imssam.me/shop/view.php?index_no=4031",
    packages = find_packages(exclude=['tests']),
    install_requires = [
        'pyserial>=3.4',
        'numpy>=1.15.4',
        'colorama>=0.4.0'],
    long_description = open('README.md').read(),
    long_description_content_type='text/markdown',
)
