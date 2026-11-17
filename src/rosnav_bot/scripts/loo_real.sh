#!/bin/bash
# Leave-one-scene-out on real ETH/UCY: synthetic-pretrained (init) vs scratch vs CV. Usage: loo_real.sh <raw_root> <init.pt> <outdir>
R=$1; INIT=$2; OUT=$3; mkdir -p $OUT
for h in eth hotel univ zara1 zara2; do
  python3 "$(dirname "$0")/train_ped_predictor.py" --real_root $R --heldout $h --epochs 12 --lr 2e-4 --init $INIT --out $OUT/ft_$h.pt 2>&1 | grep -E "CV baseline|epoch 12" | sed "s/^/$h  /"
done
