#!/bin/bash

echo "dataset = '';"

COUNTER=1
while read line
do
  name=$line
  echo "ft$COUNTER='$line';"
  echo "t$COUNTER=readResults(dataset,ft$COUNTER);"
  echo "plotResults(t$COUNTER,colors($COUNTER,:),w);hold on;"
  let COUNTER+=1
done < $1
