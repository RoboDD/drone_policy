def main():
    # update settings for residual model identification
    # settings = ...

    learner = QuadLearner(settings)
    learner.identify_residual_models()


if __name__ == "__main__":
    main()
