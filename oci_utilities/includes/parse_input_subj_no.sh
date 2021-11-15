# pass in the subject number
echo "parsing input"
echo "Passed: $1"
re='^[0-9]+$'
if ! [[ $1 =~ $re ]] ; then
   echo "error: Subject number is not a number" >&2; exit 1
fi
subject_padded=$(printf '%03d' "$1")
echo "Processing for subj: $subject_padded"
