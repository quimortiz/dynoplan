# After running the scprit, all primitives will be in dynomotions_full


# We use gdown. You can install it with: pip install gdown
# Basic command is:
# gdown --fuzzy "https://drive.google.com/file/d/1r_ecGwdfvWnVWxPsvR4d8Hjcayxg5PsB/view?usp=drive_link"

# Primitives
# unicycle1_v0 "https://drive.google.com/file/d/15dXqC_OdrI8KjaHRNakYgk9IXLtTeMtt/view?usp=drive_link"
# quadrotor_v1 "https://drive.google.com/file/d/1r_ecGwdfvWnVWxPsvR4d8Hjcayxg5PsB/view?usp=drive_link"
# quadrotor_v0 "https://drive.google.com/file/d/1j57kwE5hFgO-46LjStv_zqm6S5BFUsY8/view?usp=drive_link"
# Acrobot_v0 "https://drive.google.com/file/d/1mLiTgcpXSI9UHHss4Qt7AIsRwJPbPC2H/view?usp=drive_link"
# Roto_Pole_v0 "https://drive.google.com/file/d/1KMb4IDgucHN8uWI9YN_W07AhX59tkph_/view?usp=drive_link"
# Planar Rotor_v0 "https://drive.google.com/file/d/18kI3qXweA4RgvDxtV3vfxnfc_BhX52j8/view?usp=drive_link"
# Car1_v0 "https://drive.google.com/file/d/1TPX3c8RvMOy9hiaKL-kUE8M61OknDrDK/view?usp=drive_link"
# Unicycle 2 _v0 "https://drive.google.com/file/d/1PoK1kbiLRFq_hkv3pVWU0csNr4hap0WX/view?usp=drive_link"
# Unicycle 1 v2 "https://drive.google.com/file/d/1IvwN-e1jn5P0P1ILaVwSrUnIeBlFxhHI/view?usp=drive_link"
# Unicycle 1 v1 "https://drive.google.com/file/d/1OLuw5XICTueoZuleXOuD6vNh3PCWfHif/view?usp=drive_link"


files=( "https://drive.google.com/file/d/15dXqC_OdrI8KjaHRNakYgk9IXLtTeMtt/view?usp=drive_link" \
"https://drive.google.com/file/d/1r_ecGwdfvWnVWxPsvR4d8Hjcayxg5PsB/view?usp=drive_link" \
"https://drive.google.com/file/d/1j57kwE5hFgO-46LjStv_zqm6S5BFUsY8/view?usp=drive_link" \
"https://drive.google.com/file/d/1mLiTgcpXSI9UHHss4Qt7AIsRwJPbPC2H/view?usp=drive_link" \
"https://drive.google.com/file/d/1KMb4IDgucHN8uWI9YN_W07AhX59tkph_/view?usp=drive_link" \
"https://drive.google.com/file/d/18kI3qXweA4RgvDxtV3vfxnfc_BhX52j8/view?usp=drive_link" \
"https://drive.google.com/file/d/1TPX3c8RvMOy9hiaKL-kUE8M61OknDrDK/view?usp=drive_link" \
"https://drive.google.com/file/d/1PoK1kbiLRFq_hkv3pVWU0csNr4hap0WX/view?usp=drive_link"  \
"https://drive.google.com/file/d/1IvwN-e1jn5P0P1ILaVwSrUnIeBlFxhHI/view?usp=drive_link" \
 "https://drive.google.com/file/d/1OLuw5XICTueoZuleXOuD6vNh3PCWfHif/view?usp=drive_link" )

mkdir dynomotions_full
cd dynomotions_full

for file in ${files[*]}; do
 gdown --fuzzy $file
done
