#!/bin/bash

project=$(basename `git rev-parse --show-toplevel`)
package_name="ros-melodic-$(echo $project | perl -ne 'tr/A-Z/a-z/;s/_/-/g;print;')"
perl -i -ne "s/^(\s*project\()[^\)]+(\).*)\$/\$1${project}\$2/g;print;" CMakeLists.txt
perl -i -ne "s/^(\s*<name>).*?(<\/name.*)\$/\$1${project}\$2/g;print;" package.xml
perl -i -ne "s/^((Source|Package):\s+).*/\$1${package_name}/g;print;" debian/control
perl -i -ne "s/^\S+(\s+\(999.*)/${package_name}\$1/g;print;" debian/changelog
perl -i -ne "s/^\S+(\s+\(999.*)/${package_name}\$1/g;print;" debian/changelog
perl -i -ne "s/ros-melodic-ros-debian-template/${package_name}/g;print;" debian/rules

# Remove this repo-setup script
echo "We want to remove this repo-setup.sh so that you don't run it again"
rm -i $(readlink -f $0)


# Start a new readme file
echo -ne "* ${project} \n\nTODO\n" > README.md
