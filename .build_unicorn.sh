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

NUTTX_ROOT=${PWD}/nuttx
NUTTX_OUTDIR=${PWD}/out

NUTTX_BOARD_CONFIG=(\
		"unicorn/ap" \
		"unicorn/cp" \
		"unicorn/sp")

NUTTX_ARMTOOL=(\
		"CROSSDEV=${PWD}/prebuilts/gcc/linux/arm/bin/arm-none-eabi-" \
		"ARCROSSDEV=${PWD}/prebuilts/gcc/linux/arm/bin/arm-none-eabi-")

function usage()
{
	local i=1 j=1
	echo -e "\n----------------------------------------------------"
	echo -e "\nUnicorn build shell: [-o product path] [\${NUTTX_BOARD_CONFIG}]\n"
	echo    "	${j}. ARMTool Path:" && let j++
	for tool in ${NUTTX_ARMTOOL[*]}; do
		echo "		$i. ${tool}" && let i++
	done
	i=1
	echo -e "\n	${j}. Board Config:" && let j++
	for config in ${NUTTX_BOARD_CONFIG[*]}; do
		echo "		$i. ${config}" && let i++
	done
	echo "		(You can only compile one of the configuration through: "
	echo "-->			./build_unicorn.sh ${NUTTX_BOARD_CONFIG[0]} )"

	echo -e "\n	${j}. Default Output Directory:" && let j++
	echo -e "		${NUTTX_OUTDIR}/\${NUTTX_BOARD_CONFIG} \n"
	echo "		(You can modify this path through the -o option:"
	echo "-->			./build_unicorn.sh -o \${NUTTX_OUTDIR} "
	echo "-->			./build_unicorn.sh -o \${NUTTX_OUTDIR} ${NUTTX_BOARD_CONFIG[0]})"
	i=1
	echo -e "\n	${j}. Compile Products:" && let j++
		for config in ${NUTTX_BOARD_CONFIG[*]}; do
			echo "		$i. \${NUTTX_OUTDIR}/${config}/`echo ${config} | sed "s/\//-/g"`.fw" && let i++
		done
	echo -e "\n----------------------------------------------------"
}

function build_board()
{
	local product=`echo ${2} | sed "s/\//-/g"`.fw

	if [ -f ${product} ]; then rm -rf ${product};fi

	export ${1}
	${NUTTX_ROOT}/tools/configure.sh -o ${NUTTX_OUTDIR}/${2}/nuttx ${2} && \
	make -C nuttx O=${NUTTX_OUTDIR}/${2}/nuttx -j4

	if [ $? -ne 0 ]; then
		echo "############# build ${2} fail ##############"
		exit $?
	fi
	cp -rf ${NUTTX_OUTDIR}/${2}/nuttx/nuttx ${NUTTX_OUTDIR}/${2}/${product}
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
			build_board "${NUTTX_ARMTOOL[*]}" ${1}
			exit 0
		;;
	esac
	shift
done

for config in ${NUTTX_BOARD_CONFIG[*]}; do
	build_board "${NUTTX_ARMTOOL[*]}" ${config}
done
