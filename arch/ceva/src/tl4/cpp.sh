#!/bin/bash
#
# Copyright (C) 2018 Pinecone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

options=(\
        "-o" \
        "-MT")

function wash_source()
{
    echo `basename $1 | cut -f 1 -d '.'`
}

function rebuild_command()
{
  local args=("$@")
  local source=

  for (( i = 0; i < $#; i++ )); do
    for option in ${options[*]}; do
      if [ "${option}" == "${args[$i]}" ]; then
        let i=i+1
        source=`wash_source ${args[${i}]}`
        for (( j = 0; i < $#; j++ )); do
          if [ -z "${args[$j]}" ] || [[ ${args[$j]} =~ ^- ]] ; then
              continue
          fi
          if [[ `wash_source ${args[$j]}` == "${source}" ]] \
              && [ "${args[$j]}" != "${args[$i]}" ]; then
            args[$j]=" -include ${args[$j]}"
            echo ${args[*]}
            return
          fi
        done
      fi
    done
  done
}

command=`rebuild_command $@`
if [ -z "${command}" ]; then
  exit -1
fi

${command[*]}
