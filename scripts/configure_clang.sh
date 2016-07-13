#!/bin/bash
SCRIPTPATH=$( cd $(dirname $0) ; pwd -P )

CLANGSCRIPT=https://llvm.org/svn/llvm-project/cfe/trunk/tools/clang-format/git-clang-format
BINPATH=/usr/local/bin
CLANGBINARY=clang-format-3.6
## Install the clang auto-formating tools
sudo apt-get install ${CLANGBINARY}
sudo wget $CLANGSCRIPT -O ${BINPATH}/git-clang-format
sudo chmod +x ${BINPATH}/git-clang-format
cp ${SCRIPTPATH}/formatting/clang-format ${SCRIPTPATH}/../.clang-format

## Configure the git options
git config --add clangformat.binary "${CLANGBINARY}"
git config --add clangformat.style "file"
