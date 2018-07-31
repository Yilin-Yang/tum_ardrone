find_program(GRAPHVIZ_DOT_EXECUTABLE dot
	HINTS /usr/bin /usr/local/bin)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Graphviz DEFAULT_MSG GRAPHVIZ_DOT_EXECUTABLE)

mark_as_advanced(GRAPHVIZ_DOT_EXECUTABLE)
