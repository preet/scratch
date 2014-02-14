# libosmscout-core headers and sources
cd libosmscout && find include -name \*.h -printf %p' \\'\\n >> ../core_headers_list.dat && sed -i 's/^/$${PATH_LIBOSMSCOUT_CORE}\//' ../core_headers_list.dat && cd ..
cd libosmscout && find src -name \*.cpp -printf %p' \\'\\n >> ../core_sources_list.dat && sed -i 's/^/$${PATH_LIBOSMSCOUT_CORE}\//' ../core_sources_list.dat && cd ..

# libosmscout-import headers and sources
cd libosmscout-import && find include -name \*.h -printf %p' \\'\\n >> ../import_headers_list.dat && sed -i 's/^/$${PATH_LIBOSMSCOUT_IMPORT}\//' ../import_headers_list.dat && cd ..
cd libosmscout-import && find src -name \*.cpp -printf %p' \\'\\n >> ../import_sources_list.dat && sed -i 's/^/$${PATH_LIBOSMSCOUT_IMPORT}\//' ../import_sources_list.dat && cd ..
