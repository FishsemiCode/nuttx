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

configs=()
commands=()

ROOTDIR=$(cd "$( dirname "$0"  )" && pwd)
OUTDIR=${ROOTDIR}/out
ROOTDIR=${ROOTDIR}/nuttx

PROJECTS=(\
		"abies" \
		"banks" \
		"unicorn")

CONFIGS=(\
		"
		abies/audio
		" \
		"
		banks/audio
		banks/rpm
		banks/sensor
		" \
		"
		unicorn/ap
		unicorn/cp
		unicorn/sp
		unicorn/sim
		" \
		)

DEFAULT_PROJECTS=${PROJECTS[2]}

SYSTEM=`uname -s`
HOST_FLAG=

if [[ "${SYSTEM}" =~ "Linux" ]]; then
	HOST_FLAG=-l
elif [[ "${SYSTEM}" =~ "CYGWIN" ]]; then
	HOST_FLAG=-c
else
	echo "Unsupport OS : ${SYSTEM}"
	exit 1
fi

function get_config_list()
{
	for (( i = 0; i < ${#PROJECTS[*]}; i++)); do
		if [ $1 == ${PROJECTS[$i]} ]; then
			echo ${CONFIGS[$i]}
			break
		fi
	done
}

function usage()
{
	local i=1 j=1
	echo -e "\n----------------------------------------------------"
	echo -e "\nNuttx build shell: [-o product path] [-p project name] [ \${CONFIGS} ]\n"
	echo -e " ${i}. Projects:" && let i++
	for project in ${PROJECTS[*]}; do
		echo "      ($j) ${project}" && let j++
	done
	echo -e "\n    (Configure target project through the -p option:"
	echo -e "-->	./build.sh -p ${DEFAULT_PROJECTS})"

	j=1
	echo -e "\n ${i}. Project Board Configs:" && let i++
	for config in ${CONFIGS[*]}; do
		echo "      ($j) ${config}" && let j++
	done
	echo -e "\n    (Support compile one or more of the configurations through:"
	echo -e "-->	./build.sh `get_config_list  ${DEFAULT_PROJECTS}`)"

	echo -e "\n ${i}. Default Output Directory:" && let i++
	echo -e "       ${OUTDIR}/\${CONFIGS}"
	echo -e "\n    (Configure this path dynamically through the -o option:"
	echo "-->       ./build.sh -o \${OUTDIR}"
	echo "-->       ./build.sh -o \${OUTDIR} `get_config_list  ${DEFAULT_PROJECTS}`)"

	j=1
	echo -e "\n ${i}. Compile Products:" && let i++
		for config in ${CONFIGS[*]}; do
			echo "      ($j) \${OUTDIR}/${config}.bin" && let j++
		done
	echo -e "\n----------------------------------------------------"
}

function build_board()
{
	local product=${1}.bin
	local product_out=${OUTDIR}/${1}/nuttx

	echo -e "\nCompile Command line:\n"
	echo -e "	${ROOTDIR}/tools/configure.sh ${HOST_FLAG} -o ${product_out} ${1}"
	echo -e "	make -C ${ROOTDIR} O=${product_out} ${commands[*]}"
	echo -e "	make -C ${ROOTDIR} O=${product_out} savedefconfig\n"

	${ROOTDIR}/tools/configure.sh ${HOST_FLAG} -o ${product_out} ${1} && \
	make -C ${ROOTDIR} O=${product_out} ${commands[*]} && \
	make -C ${ROOTDIR} O=${product_out} savedefconfig

	if [ $? -ne 0 ]; then
		echo "Error: ############# build ${1} fail ##############"
		exit $?
	fi

	if [ -f ${product_out}/defconfig ]; then
		cp ${product_out}/defconfig ${ROOTDIR}/configs/${1}
	fi

	if [ -f ${product_out}/nuttx.bin ]; then
		cp -f ${product_out}/nuttx.bin ${OUTDIR}/${product}
	fi
}

while [ ! -z "$1" ]; do
	case $1 in
		-o )
			shift
			OUTDIR=$1
			;;
		-p )
			shift
			config_list=`get_config_list $1`

			if [ "${config_list[*]}" ]; then
				for config in ${config_list[*]}; do
					configs[${#configs[@]}]=${config}
				done
			else
				echo "Error: Unable to find the board or configurations from $1"
				exit 2
			fi
			;;
		-h )
			usage
			exit 0
			;;
		* )

			find_config=
			if [ -d "${ROOTDIR}/configs/$1" ]; then
				configs[${#configs[@]}]=$1
				find_config=true
			fi


			if [ "${find_config}" == "" ];then
				commands[${#commands[@]}]=$1
			fi
			;;
	esac
	shift
done

if [ -n "${configs[*]}" ]; then
	for config in ${configs[*]}; do
		build_board ${config}
	done
	exit $?
else
	configs=`get_config_list ${DEFAULT_PROJECTS}`
	for config in ${configs[*]}; do
		build_board ${config}
	done
fi
