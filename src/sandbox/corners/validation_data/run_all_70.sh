for i in {70..89}
do
	mkdir ${i}_$((i + 1))
   ./corners "flattened_input_15/$i.png" "flattened_input_15/$((i + 1)).png" "${i}_$((i + 1))" 2> "${i}_$((i + 1))/candidates.txt"
done