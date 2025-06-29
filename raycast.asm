; =============================================================================
;        WOLF3D-STYLE RAYCASTING ENGINE for MS-DOS by Viper Droid!
; =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
; To Assemble and Run:
; 1. Install NASM Assembler and DOSBox.
; 2. nasm raycast.asm -f bin -o raycast.com
; 3. Run raycast.com inside DOSBox.
;
; Controls:
;   Up Arrow:    Move Forward
;   Down Arrow:  Move Backward
;   Left Arrow:  Turn Left
;   Right Arrow: Turn Right
;   ESC:         Exit
; =============================================================================

ORG 100h

SECTION .text

_start:
    call    Setup
    jmp     MainLoop

; =============================================================================
;   CONSTANTS AND SETTINGS
; =============================================================================
SCREEN_WIDTH    equ 320
SCREEN_HEIGHT   equ 200
MAP_WIDTH       equ 16
MAP_HEIGHT      equ 16

; Fixed-point math settings. We use 8 bits for the fractional part.
FIXED_SHIFT     equ 8
FIXED_ONE       equ 1 << FIXED_SHIFT  ; Represents 1.0

; Player movement settings
MOVE_SPEED      equ 80  ; Fixed-point value for speed
TURN_SPEED      equ 8   ; Angle units per frame

; There are 256 angles in our full circle. This is easier than 360 degrees.
ANGLES          equ 256
ANGLE_90        equ ANGLES / 4
ANGLE_180       equ ANGLES / 2
ANGLE_270       equ ANGLES * 3 / 4
ANGLE_360       equ ANGLES

; Field of View (FOV). 64 angles = 90 degree FOV.
FOV             equ 64

; =============================================================================
;   DATA AND VARIABLES
; =============================================================================
player_x:       dw FIXED_ONE * 2    ; Player X position (fixed-point)
player_y:       dw FIXED_ONE * 2    ; Player Y position (fixed-point)
player_angle:   dw 0                ; Player angle (0-255)

; Trigonometry lookup tables. These are the core of the engine's performance.
; They will be filled by the GenerateTables procedure.
sine_table:     times ANGLES dw 0
cosine_table:   times ANGLES dw 0
fisheye_table:  times SCREEN_WIDTH dw 0 ; For correcting distortion

; The 2D map of the world. 1 = Wall, 0 = Empty Space
map: db 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
     db 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1
     db 1,0,1,1,0,0,1,0,1,0,1,1,1,1,0,1
     db 1,0,0,1,0,1,1,0,0,0,0,0,0,1,0,1
     db 1,0,1,0,0,0,1,0,1,0,1,0,0,1,0,1
     db 1,0,1,1,1,0,0,0,1,0,1,1,0,1,0,1
     db 1,0,0,0,1,0,1,1,1,0,0,0,0,0,0,1
     db 1,1,1,0,1,0,0,0,0,0,1,1,1,1,0,1
     db 1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1
     db 1,0,1,1,1,1,1,0,1,1,1,0,1,1,0,1
     db 1,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1
     db 1,1,1,1,1,0,1,1,1,0,1,0,1,1,0,1
     db 1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,1
     db 1,0,1,0,1,0,1,0,1,1,1,1,1,1,0,1
     db 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1
     db 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
     
; =============================================================================
;   MAIN PROGRAM AND SETUP
; =============================================================================
Setup:
    ; Set video mode to 320x200x256
    mov     ax, 13h
    int     10h

    ; Generate the sine/cosine tables. This is done once.
    call    GenerateTables
    ret

MainLoop:
    call    HandleInput
    call    DrawScreen
    call    WaitRetrace ; A simple delay to cap framerate
    jmp     MainLoop

; =============================================================================
;   INPUT HANDLING
; =============================================================================
HandleInput:
    mov     ah, 1       ; Check for key press
    int     16h
    jz      .no_key     ; No key waiting

    mov     ah, 0       ; Get key
    int     16h

    cmp     ah, 1       ; ESC key scan code
    je      ExitProgram
    cmp     ah, 72      ; Up arrow
    je      .move_fwd
    cmp     ah, 80      ; Down arrow
    je      .move_back
    cmp     ah, 75      ; Left arrow
    je      .turn_left
    cmp     ah, 77      ; Right arrow
    je      .turn_right
    jmp     .no_key

.move_fwd:
    ; Calculate movement vector: dx = cos(angle)*speed, dy = sin(angle)*speed
    mov     bx, [player_angle]
    mov     ax, [cosine_table + bx*2] ; AX = cos(angle)
    imul    word [move_speed]         ; DX:AX = cos(angle) * speed
    sar     dx, FIXED_SHIFT           ; DX = fixed-point delta_x
    mov     di, dx
    
    mov     ax, [sine_table + bx*2]   ; AX = sin(angle)
    imul    word [move_speed]         ; DX:AX = sin(angle) * speed
    sar     dx, FIXED_SHIFT           ; DX = fixed-point delta_y

    ; Add to player position
    add     word [player_x], di
    add     word [player_y], dx
    jmp     .no_key

.move_back:
    mov     bx, [player_angle]
    mov     ax, [cosine_table + bx*2]
    imul    word [move_speed]
    sar     dx, FIXED_SHIFT
    mov     di, dx
    
    mov     ax, [sine_table + bx*2]
    imul    word [move_speed]
    sar     dx, FIXED_SHIFT

    sub     word [player_x], di
    sub     word [player_y], dx
    jmp     .no_key
    
.turn_left:
    sub     word [player_angle], TURN_SPEED
    and     word [player_angle], ANGLES-1 ; Wrap around 0-255
    jmp     .no_key

.turn_right:
    add     word [player_angle], TURN_SPEED
    and     word [player_angle], ANGLES-1 ; Wrap around 0-255

.no_key:
    ret

ExitProgram:
    ; Restore text mode and exit to DOS
    mov     ax, 3
    int     10h
    mov     ax, 4C00h
    int     21h

; =============================================================================
;   RENDERING ROUTINES
; =============================================================================
DrawScreen:
    ; This is the main raycasting loop
    mov     di, 0 ; DI = current screen column (0-319)
.cast_loop:
    cmp     di, SCREEN_WIDTH
    jge     .cast_done

    ; Step 1: Calculate the angle for this ray
    mov     ax, [player_angle]
    mov     bx, di
    sub     bx, SCREEN_WIDTH / 2    ; BX = column - center
    add     ax, [fisheye_table + bx*2]
    and     ax, ANGLES - 1          ; AX = ray_angle, wrapped
    
    ; Step 2: Get the ray's direction vector (fixed-point)
    mov     si, ax
    mov     bx, [cosine_table + si*2]   ; BX = ray_dx
    mov     cx, [sine_table + si*2]     ; CX = ray_dy

    ; Step 3: Use DDA to find wall intersection
    ; Initial position for DDA is player's position
    mov     si, [player_x] ; SI = ray_x (fixed-point)
    mov     bp, [player_y] ; BP = ray_y (fixed-point)
    
    ; DDA loop
    mov     word [distance], 0
.dda_loop:
    ; Move ray forward one small step
    add     si, bx  ; ray_x += ray_dx
    add     bp, cx  ; ray_y += ray_dy
    add     word [distance], 2 ; Approximating distance
    
    ; Get current map cell coordinates (integer)
    mov     ax, si
    shr     ax, FIXED_SHIFT ; AX = map_x
    mov     dx, bp
    shr     dx, FIXED_SHIFT ; DX = map_y
    
    ; Calculate map index: index = y * MAP_WIDTH + x
    imul    dx, MAP_WIDTH
    add     dx, ax
    
    ; Check if we hit a wall
    cmp     byte [map + dx], 1
    jne     .dda_loop

    ; Step 4: We hit a wall. Calculate distance and wall height.
    mov     ax, [distance]
    ; Correct for fisheye distortion
    mov     bx, di
    sub     bx, SCREEN_WIDTH / 2
    mov     dx, [cosine_table + bx*2 + FOV/2*2] ; Cosine of angle difference
    imul    dx
    shrd    ax, dx, FIXED_SHIFT
    
    ; Calculate wall height (inverse relationship with distance)
    ; wall_height = (some_constant * screen_height) / distance
    mov     dx, SCREEN_HEIGHT * 64 ; A constant for scaling
    ; If distance is 0, avoid division by zero
    cmp     ax, 0
    je      .draw_column
    div     ax  ; AX = wall_height

    ; Clamp height to screen height
    cmp     ax, SCREEN_HEIGHT
    jle     .height_ok
    mov     ax, SCREEN_HEIGHT
.height_ok:

    ; Step 5: Calculate where to draw the vertical line on screen
    mov     cx, SCREEN_HEIGHT
    sub     cx, ax      ; CX = empty space
    shr     cx, 1       ; CX = top_y
    mov     bx, cx
    add     bx, ax      ; BX = bottom_y

    ; Step 6: Draw the column
    push    di ; Save screen column
    ; Draw ceiling
    mov     al, 25 ; Ceiling color (light blue)
    call    DrawVerticalLine
    
    ; Draw wall
    mov     cx, bx      ; New top_y is old bottom_y
    mov     bx, [esp]   ; Retrieve screen column
    push    bx
    mov     bx, cx
    mov     cl, [esp+2] ; old top_y
    sub     bx, cx      ; BX = wall height
    add     cl, bl      ; CL = new bottom_y
    mov     bx, cx      ; new top_y
    mov     cx, cl      ; new bottom_y
    
    ; Shade wall based on distance
    mov     al, 7 ; Wall color (light grey)
    mov     ah, [distance]
    shr     ah, 3
    sub     al, ah
    jae     .color_ok
    mov     al, 0 ; Black if too dark
.color_ok:

    call    DrawVerticalLine

    ; Draw floor
    mov     al, 2 ; Floor color (dark green)
    mov     cx, SCREEN_HEIGHT
    call    DrawVerticalLine
    pop     di ; Restore screen column

.draw_column:
    inc     di
    jmp     .cast_loop

.cast_done:
    ret
distance: dw 0

DrawVerticalLine:
    ; INPUT: DI=column, CX=y_start, BX=y_end, AL=color
    pusha
    mov     ah, al
    mov     dx, cx ; Current Y
.vline_loop:
    cmp     dx, bx
    jge     .vline_done
    
    ; Calculate pixel offset: offset = y*320 + x
    mov     ax, dx
    mov     cx, 320
    mul     cx
    add     ax, di
    push    ax
    
    ; Draw pixel
    mov     ax, 0A000h
    mov     es, ax
    pop     di
    mov     [es:di], al

    ; Next pixel
    popa
    pusha
    inc     dx
    jmp     .vline_loop

.vline_done:
    popa
    ret

; =============================================================================
;   UTILITY AND MATH ROUTINES
; =============================================================================
WaitRetrace:
    ; Wait for vertical retrace to begin, prevents screen tearing
    mov     dx, 3DAh
.wait1:
    in      al, dx
    and     al, 8
    jz      .wait1
.wait2:
    in      al, dx
    and     al, 8
    jnz     .wait2
    ret

GenerateTables:
    ; This procedure generates sine, cosine, and fisheye correction tables
    ; using fixed-point math approximations. A real C program would use floating point.
    ; This is a complex part, involving Taylor series or similar approximations,
    ; but here we will use a simpler (less accurate) incremental method.

    ; -- Fisheye Table Generation --
    ; Calculates cos(angle_difference) for each screen column
    mov     cx, 0
.fisheye_loop:
    cmp     cx, SCREEN_WIDTH
    jge     .fisheye_done

    ; angle = arctan((column - SCREEN_WIDTH/2) / projection_plane_dist)
    ; This is very complex. We'll pre-calculate a simplified version.
    ; For this demo, let's use a pre-calculated table for speed.
    ; Realistically, this requires more complex math than is feasible here.
    ; For now, we'll just set it up.
    mov     bx, cx
    sub     bx, SCREEN_WIDTH / 2    ; -160 to 159
    ; The angle is (bx * FOV) / SCREEN_WIDTH
    mov     ax, bx
    imul    word [fov_angle_step]
    sar     ax, FIXED_SHIFT
    add     ax, FOV/2 ; Center it
    and     ax, ANGLES-1
    mov     [fisheye_table + cx*2], ax
    
    inc     cx
    jmp     .fisheye_loop
.fisheye_done:

    ; -- Sine/Cosine Table Generation using incremental rotation --
    ; Start with angle 0: cos(0)=1, sin(0)=0
    mov     word [cosine_table], FIXED_ONE
    mov     word [sine_table], 0
    
    ; Pre-calculate cos and sin of a small angle (1/256th of a circle)
    ; These values are hardcoded for simplicity. They are sin(2*pi/256) and cos(2*pi/256)
    ; in fixed-point representation.
    mov     ax, 6393 ; cos(small_angle) in Q8 format (approx 0.998)
    mov     bx, 156 ; sin(small_angle) in Q8 format (approx 0.0245)

    mov     cx, 1
.gen_loop:
    cmp     cx, ANGLES
    jge     .gen_done
    
    ; Use rotation formula:
    ; cos(A+B) = cos(A)cos(B) - sin(A)sin(B)
    ; sin(A+B) = sin(A)cos(B) + cos(A)sin(B)
    
    ; Previous values
    mov     dx, [cosine_table + (cx-1)*2] ; cos(A)
    mov     si, [sine_table + (cx-1)*2]   ; sin(A)
    
    ; Calculate new cosine
    push    ax ; save cos(B)
    imul    dx ; cos(A)*cos(B)
    mov     di, ax ; store result
    mov     ax, bx ; sin(B)
    imul    si ; sin(A)*sin(B)
    sub     di, ax ; cos(A)cos(B) - sin(A)sin(B)
    sar     di, FIXED_SHIFT ; scale down
    mov     [cosine_table + cx*2], di
    pop     ax ; restore cos(B)

    ; Calculate new sine
    push    ax ; save cos(B)
    imul    si ; sin(A)*cos(B)
    mov     di, ax ; store result
    mov     ax, bx ; sin(B)
    imul    dx ; cos(A)*sin(B)
    add     di, ax ; sin(A)cos(B) + cos(A)sin(B)
    sar     di, FIXED_SHIFT
    mov     [sine_table + cx*2], di
    pop     ax

    inc     cx
    jmp     .gen_loop

.gen_done:
    ret
fov_angle_step dw (FOV * FIXED_ONE) / SCREEN_WIDTH
