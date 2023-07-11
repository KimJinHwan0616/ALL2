# 정확도 = 정답 수 / 전체 데이터 수 * 100
# 배치 크기 = m    배치 수 = n
# 전체 데이터 = m x n    출력 클래스 = a

def compute_accuracy(model, data_loader, device):
    with torch.no_grad():
        pred_correct = 0    # 예측 정확도
        total_dataset = 0    # 전체 데이터 수

        for i, (features, targets) in enumerate(data_loader):
            features = features.to(device)
            targets = targets.to(device)
            # targets = tensor( [라벨1, 라벨2, ..., 라벨m] )

            pred = model(features)    # 예측값
            # pred = tensor([
            #     [출력층 노드1], ..., [출력층 노드a]    # 1
            #                   ...
            #     [출력층 노드1], ..., [출력층 노드a]    # m
            #     ])
            _, predicted_labels = torch.max(pred, axis=1)    # 예측 최대
            # torch.max(pred, axis=1) = (
            #     values=tensor( [최대값1, ..., 최대값m] ), indices=tensor( [인덱스1, ..., 인덱스m] )
            #     )
            # predicted_labels = tensor( [인덱스1, ..., 인덱스m] )

            total_dataset += len(targets)    # m + ... + m = m x n = 전체 데이터

            pred_correct += (predicted_labels == targets).sum()    # 인덱스 = 라벨(원핫인코딩)
            # (predicted_labels == targets).sum() = tensor(정답 수)
            # 정답 수 + 오답 수 = m

    return pred_correct / total_dataset * 100    # 정확도(%)