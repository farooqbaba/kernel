# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2013 KYOCERA Corporation
ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor.o
obj-$(CONFIG_S5K3L1YX) += s5k3l1yx.o
obj-$(CONFIG_IMX135) += imx135.o
obj-$(CONFIG_OV8825) += ov8825.o
obj-$(CONFIG_OV2720) += ov2720.o
obj-$(CONFIG_OV9724) += ov9724.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_THP7212) += thp7212.o
obj-$(CONFIG_THP7212) += thp7212_fw.o
obj-$(CONFIG_S5K8AAY) += s5k8aay.o
obj-$(CONFIG_SP1628) += sp1628.o
obj-$(CONFIG_GC0339) += gc0339.o
