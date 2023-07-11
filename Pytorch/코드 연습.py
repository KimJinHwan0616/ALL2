import time

def train(
        model, epochs,
        train_loader, valid_loader, test_loader,
        optimizer, device,
        minibatch_interval=50,  # 배치 수 간격
        scheduler=None, scheduler_on='valid_acc'
):
    start_time = time.time()  # 시작(현재) 시간
    minibatch_cost_list, train_accuracy_list, valid_accuracy_list = [], [], []

    for epoch in range(epochs):
        model.train()    # 모델 학습(드롭아웃, 배치 정규화 포함)

        avg_cost = 0    # 미니 배치 오차

        for batch_index, (features, targets) in enumerate(train_loader):
            # GPU
            features = features.to(device)
            targets = targets.to(device)

            # 순전파
            pred = model(features)    # 예측값
            minibatch_cost = cost_function(pred, targets)    # 오차

            # 역전파
            optimizer.zero_grad()  # 기울기 초기화(기울기=0)
            minibatch_cost.backward()  # 미분(비용 함수 → 기울기)
            optimizer.step()  # 파라미터 업데이트

            minibatch_cost_list.append(minibatch_cost.item())

            avg_cost += minibatch_cost / len(train_loader)

            if not batch_index % minibatch_interval:
                print(f'에포크: {epoch + 1:3d}/{epochs:3d} '
                      f'| 배치 수 {batch_index:4d}/{len(train_loader):4d} '
                      f'| 오차: {minibatch_cost:.4f}'
                      )

        model.eval()
        with torch.no_grad():
            train_accuracy = compute_accuracy(model, train_loader, device=device)
            valid_accuracy = compute_accuracy(model, valid_loader, device=device)
            print(f'훈련 정확도: {train_accuracy :.2f}% '
                  f'| 검증 정확도: {valid_accuracy :.2f}%'
                  f'| 미니 배치 오차: {avg_cost:.4f}'
                  )

            train_accuracy_list.append(train_accuracy.item())
            valid_accuracy_list.append(valid_accuracy.item())

        end_time = (time.time() - start_time) / 60  # 종료 시간
        print(f'누적 학습 시간: {end_time:.2f} 분')
        print()

        if scheduler is not None:

            if scheduler_on == 'valid_accuracy':
                scheduler.step(valid_accuracy_list[-1])
            elif scheduler_on == 'minibatch_cost':
                scheduler.step(minibatch_cost_list[-1])
            else:
                raise ValueError(f'Invalid `scheduler_on` choice.')

    test_accuracy = compute_accuracy(model, test_loader, device=device)
    print(f'테스트 정확도: {test_accuracy :.2f}%')

    end_time = (time.time() - start_time) / 60
    print(f'전체 학습 시간: {end_time:.2f} 분')

    return minibatch_cost_list, train_accuracy_list, valid_accuracy_list