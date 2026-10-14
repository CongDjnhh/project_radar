# invoke SourceDir generated makefile for mmw_mss.per4ft
mmw_mss.per4ft: .libraries,mmw_mss.per4ft
.libraries,mmw_mss.per4ft: package/cfg/mmw_mss_per4ft.xdl
	$(MAKE) -f C:\Users\Admin\workspace_ccstheia\oob_1843_mss/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Admin\workspace_ccstheia\oob_1843_mss/src/makefile.libs clean

