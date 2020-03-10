define re
    monitor reset halt
    load
    monitor reset init
end

define blog
    x/13xb 0x40066000
    p _debugBuffer
    call db_init
end

define reload
    monitor reset halt
    load
    monitor reset init
end

set print elements 350

target extended-remote :3333

reload
b _assert_failed

