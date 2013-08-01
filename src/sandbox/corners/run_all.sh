for i in {70..89}
do
   ./corners "flattened_input_15/$i.png" "flattened_input_15/$((i + 1)).png" "${i}_$((i + 1)).png"
done