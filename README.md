###  SkiROS2 template library

Template of a custom package for [skiros2](https://github.com/RVMI/skiros2).

### Configure steps

The renaming process below uses a common Perl tool to rename files. It can be installed with
```
sudo apt install rename
```

* Replace word "template" with your package name:
```
cd skiros2_template_lib
# Rename inside files
git grep -l 'template' | xargs sed -i 's/template/my_package_name/g'
# Rename files
find . -depth -execdir rename 's/template/my_package_name/' '{}' \;
```
* Replace word "xyz" with your robot name:
```
git grep -l 'xyz' | xargs sed -i 's/xyz/my_robot/g'
find . -depth -execdir rename 's/xyz/my_robot_name/' '{}' \;
```
* Build your workspace
* Source your workspace
* Launch main.launch
```
roslaunch skiros2_<my_package_name>_lib main.launch
```

