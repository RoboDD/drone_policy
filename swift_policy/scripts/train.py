def main():
    # assemble settings object to train a new policy from scratch
    # settings = ...

    learner = QuadLearner(settings)
    learner.train()


if __name__ == "__main__":
    main()
