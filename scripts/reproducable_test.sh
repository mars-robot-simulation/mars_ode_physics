error=0

for ((i=1; i<=$1; ++i )) ; 
do
    ./reproducable_test &> /dev/null
    cp reproducable_test_log.csv $i.csv
done

for ((i=2; i<=$1; ++i )) ; 
do
    diff 1.csv $i.csv
    retVal=$?
    rm $i.csv
    if [ $retVal -ne 0 ]; then
        error=1
        echo "Error"
    fi
done
rm 1.csv

exit $error
