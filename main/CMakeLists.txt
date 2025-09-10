set(srcs "main.c"
    "board.c")


idf_component_register(
    SRCS
        "main.c"
        "uart_rx.c"        
        "board.c"          # <-- add this
        INCLUDE_DIRS  ".")
       
