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

config_array=()
command_array=()

ROOTDIR=$(cd "$( dirname "$0"  )" && pwd)

NUTTX_ROOT=${ROOTDIR}/nuttx
NUTTX_OUTDIR=${PWD}/out

NUTTX_BOARD_CONFIG=(\
		"unicorn/ap" \
		"unicorn/cp" \
		"unicorn/sp")

SYSTEM=`uname -s`
HOST_FLAG=

if [[ "${SYSTEM}" =~ "Linux" ]]; then
	SYSTEM=linux
	HOST_FLAG=-l
elif [[ "${SYSTEM}" =~ "CYGWIN" ]]; then
	SYSTEM=windows
	HOST_FLAG=-c
else
	echo "Unsupport OS : ${SYSTEM}"
	exit 1
fi

NUTTX_UNICORN_ENV=(\
		"CROSSDEV=${ROOTDIR}/prebuilts/gcc/${SYSTEM}/arm/bin/arm-none-eabi-" \
		"ARCROSSDEV=${ROOTDIR}/prebuilts/gcc/${SYSTEM}/arm/bin/arm-none-eabi-")

function usage()
{
	local i=1 j=1
	echo -e "\n----------------------------------------------------"
	echo -e "\nUnicorn build shell: [-o product path] [\${NUTTX_BOARD_CONFIG}]\n"
	echo    "	${j}. ARMTool Path:" && let j++
	for tool in ${NUTTX_UNICORN_ENV[*]}; do
		echo "		$i. ${tool}" && let i++
	done
	i=1
	echo -e "\n	${j}. Board Config:" && let j++
	for config in ${NUTTX_BOARD_CONFIG[*]}; do
		echo "		$i. ${config}" && let i++
	done
	echo "		(You can only compile one of the configuration through:"
	echo "-->			./build_unicorn.sh ${NUTTX_BOARD_CONFIG[0]} )"

	echo -e "\n	${j}. Default Output Directory:" && let j++
	echo -e "		${NUTTX_OUTDIR}/\${NUTTX_BOARD_CONFIG}\n"
	echo "		(You can modify this path through the -o option:"
	echo "-->			./build_unicorn.sh -o \${NUTTX_OUTDIR}"
	echo "-->			./build_unicorn.sh -o \${NUTTX_OUTDIR} ${NUTTX_BOARD_CONFIG[0]})"
	i=1
	echo -e "\n	${j}. Compile Products:" && let j++
		for config in ${NUTTX_BOARD_CONFIG[*]}; do
			echo "		$i. \${NUTTX_OUTDIR}/`echo ${config} | sed "s/\//-/g"`.fw" && let i++
		done
	echo -e "\n----------------------------------------------------"
}

function build_board()
{
	local product=`echo ${2} | sed "s/\//-/g"`.fw
	local product_out=${NUTTX_OUTDIR}/${2}/nuttx

	export ${1}
	echo -e "\nCompile Command line:\n"
	echo -e "	${NUTTX_ROOT}/tools/configure.sh ${HOST_FLAG} -o ${product_out} ${2}"
	echo -e "	make -C ${NUTTX_ROOT} O=${product_out} ${command_array[*]}"
	echo -e "	make -C ${NUTTX_ROOT} O=${product_out} savedefconfig\n"

	${NUTTX_ROOT}/tools/configure.sh ${HOST_FLAG} -o ${product_out} ${2} && \
	make -C ${NUTTX_ROOT} O=${product_out} ${command_array[*]} && \
	make -C ${NUTTX_ROOT} O=${product_out} savedefconfig

	if [ $? -ne 0 ]; then
		echo "############# build ${2} fail ##############"
		exit $?
	fi

	if [ -f ${product_out}/defconfig ]; then
		cp ${product_out}/defconfig ${NUTTX_ROOT}/configs/${2}
	fi

	if [ -f ${product_out}/nuttx.bin ]; then
		cp -f ${product_out}/nuttx.bin ${NUTTX_OUTDIR}/${product}
	fi
}

while [ ! -z "$1" ]; do
	case $1 in
		-o )
			shift
			NUTTX_OUTDIR=$1
			;;
		-h )
			usage
			exit 0
			;;
		* )

			find_config=
			for config in ${NUTTX_BOARD_CONFIG[*]}; do
				if [ $1 == ${config} ]; then
					config_array[${#config_array[@]}]=$1
					find_config=true
					break
				fi
			done

			if [ "${find_config}" == "" ];then
				command_array[${#command_array[@]}]=$1
			fi
			;;
	esac
	shift
done

if [ -n "${config_array[*]}" ]; then
	for config in ${config_array[*]}; do
		build_board "${NUTTX_UNICORN_ENV[*]}" ${config}
	done
	exit $?
fi

for config in ${NUTTX_BOARD_CONFIG[*]}; do
	build_board "${NUTTX_UNICORN_ENV[*]}" ${config}
done
