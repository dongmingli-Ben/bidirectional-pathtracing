EXEC=./pathtracer

cd build

# part 4
# $EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.2 -d 4.1 -r 480 360 -f dragon_lr0.2_fd4.1_m5_l16.png ../dae/sky/CBdragon.dae
# $EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.2 -d 4.4 -r 480 360 -f dragon_lr0.2_fd4.4_m5_l16.png ../dae/sky/CBdragon.dae
# $EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.2 -d 4.7 -r 480 360 -f dragon_lr0.2_fd4.7_m5_l16.png ../dae/sky/CBdragon.dae
# $EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.2 -d 5.2 -r 480 360 -f dragon_lr0.2_fd5.2_m5_l16.png ../dae/sky/CBdragon.dae
$EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.1 -d 5.1 -r 480 360 -f dragon_lr0.1_fd5.1_m5_l16.png ../dae/sky/CBdragon.dae
$EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.2 -d 5.1 -r 480 360 -f dragon_lr0.2_fd5.1_m5_l16.png ../dae/sky/CBdragon.dae
$EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.3 -d 5.1 -r 480 360 -f dragon_lr0.3_fd5.1_m5_l16.png ../dae/sky/CBdragon.dae
$EXEC -t 2 -s 512 -a 64 0.05 -l 4 -m 5 -b 0.6 -d 5.1 -r 480 360 -f dragon_lr0.6_fd5.1_m5_l16.png ../dae/sky/CBdragon.dae

# part 2
# $EXEC -t 2 -s 512 -a 64 0.05 -l 1 -m 5 -r 480 360 -f dragon_l1_m5_a0.5.png ../dae/sky/CBdragon_microfacet_au.dae
# $EXEC -t 2 -s 512 -a 64 0.05 -l 1 -m 5 -r 480 360 -f dragon_l1_m5_a0.25.png ../dae/sky/CBdragon_microfacet_au.dae
# $EXEC -t 2 -s 512 -a 64 0.05 -l 1 -m 5 -r 480 360 -f dragon_l1_m5_a0.05.png ../dae/sky/CBdragon_microfacet_au.dae
# $EXEC -t 2 -s 512 -a 64 0.05 -l 1 -m 5 -r 480 360 -f dragon_l1_m5_a0.005.png ../dae/sky/CBdragon_microfacet_au.dae
# $EXEC -t 2 -s 256 -a 64 0.05 -l 1 -m 5 -r 480 360 -f bunny_l1_m5_importance.png ../dae/sky/CBbunny_microfacet_cu.dae
# $EXEC -t 2 -s 256 -a 64 0.05 -l 1 -m 5 -r 480 360 -f bunny_l1_m5_uniform.png ../dae/sky/CBbunny_microfacet_cu.dae
# Zirconium
# <microfacet>
#   <alpha>0.1</alpha>
#   <eta>2.6389 2.5220 2.3916</eta>
#   <k>1.9652 1.8625 1.6960</k>
# </microfacet>
# $EXEC -t 2 -s 128 -a 16 0.05 -l 4 -m 5 -r 480 360 -f dragon_Zirconium_l4_m5_a0.1.png ../dae/sky/CBdragon_microfacet_au.dae

# part 3
# $EXEC -t 2 -s 4 -l 64 -m 1 -e ../exr/field.exr -f bunny_unlit_s4_l64_m1_efield.png ../dae/sky/bunny_unlit.dae
# $EXEC -t 2 -s 4 -l 64 -m 1 -e ../exr/field.exr -f bunny_microfacet_s4_l64_m1_efield.png ../dae/sky/bunny_microfacet_cu_unlit.dae
# $EXEC -t 2 -s 4 -l 64 -m 1 -e ../exr/field.exr -f bunny_unlit_uniform_s4_l64_m1_efield.png ../dae/sky/bunny_unlit.dae
# $EXEC -t 2 -s 4 -l 64 -m 1 -e ../exr/field.exr -f bunny_microfacet_uniform_s4_l64_m1_efield.png ../dae/sky/bunny_microfacet_cu_unlit.dae

./build/pathtracer -t 2 -s 4 -l 4 -m 5 -r 480 360 ./dae/sky/CBspheres.dae
./build/pathtracer -t 2 -s 4 -l 4 -m 5 -r 480 360 dae/sky/CBspheres_lambertian.dae