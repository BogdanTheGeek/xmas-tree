all : flash

TARGET:=main
CH32FUN ?= ./ch32fun/ch32fun

TARGET_MCU?=CH32V003
include $(CH32FUN)/ch32fun.mk

flash : cv_flash
clean : cv_clean

