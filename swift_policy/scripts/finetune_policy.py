def main():
    # Update settings for finetuning operation
    # settings = ...


    learner = QuadLearner(settings)
    learner.train()


if __name__ == "__main__":
    main()
