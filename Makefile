all: checkmakefiles
	@cd src && $(MAKE)

clean: checkmakefiles
	@cd src && $(MAKE) clean

cleanall: checkmakefiles
	@cd src && $(MAKE) MODE=release clean
	@cd src && $(MAKE) MODE=debug clean
	@rm -f src/Makefile

makefiles:
	@cd src && opp_makemake --make-so -f --deep -o lte -O out -KINET_PROJ=../../inet -KVEINS_PROJ=../../veins-f2md -KVEINS_INET_PROJ=../../veins-f2md/subprojects/veins_inet3 -DINET_IMPORT -DVEINS_PROJ -DVEINS_INET_PROJ -I. -I$$\(INET_PROJ\)/src -I$$\(VEINS_PROJ\)/src -I$$\(VEINS_INET_PROJ\)/src -L$$\(INET_PROJ\)/out/$$\(CONFIGNAME\)/src -L$$\(VEINS_PROJ\)/out/$$\(CONFIGNAME\)/src -L$$\(VEINS_INET_PROJ\)/out/$$\(CONFIGNAME\)/src -lINET -lveins -lveins_inet

checkmakefiles:
	@if [ ! -f src/Makefile ]; then \
	echo; \
	echo '======================================================================='; \
	echo 'src/Makefile does not exist. Please use "make makefiles" to generate it!'; \
	echo '======================================================================='; \
	echo; \
	exit 1; \
	fi